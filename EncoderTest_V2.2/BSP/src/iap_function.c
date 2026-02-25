#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "MKV30F12810.h"
#include "fsl_flash.h"
#include "iap_function.h"
#include "uart_config.h"


/* 协议常量 */
#define IAP_HEADER1 (0x55u)
#define IAP_HEADER2 (0xAAu)

/* Flash 驱动上下文 */
static flash_config_t s_flashDriver;

/* 是否处于 IAP 模式（供中断判断是否喂协议解析器） */
volatile bool g_iap_mode = false;

/* =========================================================================
 * CRC16 Modbus（按字节累加）
 * ========================================================================= */
static uint16_t IAP_Crc16Modbus(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = (crc >> 1);
        }
    }
    return crc;
}

/* =========================================================================
 * 协议解析（在 UART ISR 中逐字节喂入）
 * ========================================================================= */
typedef enum {
    ST_WAIT_55 = 0,
    ST_WAIT_AA,
    ST_LEN_L,
    ST_LEN_H,
    ST_ADDR0,
    ST_ADDR1,
    ST_ADDR2,
    ST_ADDR3,
    ST_DATA,
    ST_CRC0,
    ST_CRC1
} IapParseState;

typedef struct {
    IapParseState state;
    uint16_t len;
    uint32_t addr;
    uint16_t crc_calc;
    uint16_t crc_rx;
    uint16_t data_idx;
} IapParser;

/* 接收数据缓冲（最大一帧数据段） */
static uint8_t s_iap_rx_buf[IAP_PACKET_MAX];
static IapParser s_parser;

/* 解析结果标志与数据（供主循环处理） */
volatile bool s_frame_ready = false;
static uint16_t s_frame_len = 0;
static uint32_t s_frame_addr = 0;

static inline void IAP_ParserReset(void)
{
    s_parser.state = ST_WAIT_55;
    s_parser.len = 0;
    s_parser.addr = 0;
    s_parser.crc_calc = 0xFFFF;
    s_parser.crc_rx = 0;
    s_parser.data_idx = 0;
}

/**
 * @brief UART1 中断调用：喂一个字节进入解析器
 */
void IAP_ProtocolFeedByte(uint8_t b)
{
    if (s_frame_ready) {
        /* 上一帧尚未处理完，忽略更多数据 */
        return;
    }

    switch (s_parser.state) {
        case ST_WAIT_55:
            if (b == IAP_HEADER1) {
                s_parser.state = ST_WAIT_AA;
                s_parser.crc_calc = 0xFFFF;
                s_parser.len = 0;
                s_parser.addr = 0;
                s_parser.data_idx = 0;
            }
            break;

        case ST_WAIT_AA:
            if (b == IAP_HEADER2) {
                s_parser.state = ST_LEN_L;
            } else {
                s_parser.state = ST_WAIT_55;
            }
            break;

        case ST_LEN_L:
            s_parser.len = b;
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            s_parser.state = ST_LEN_H;
            break;

        case ST_LEN_H:
            s_parser.len |= ((uint16_t)b << 8);
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            if (s_parser.len > IAP_PACKET_MAX) {
                IAP_ParserReset();
            } else {
                s_parser.state = ST_ADDR0;
            }
            break;

        case ST_ADDR0:
            s_parser.addr = b;
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            s_parser.state = ST_ADDR1;
            break;

        case ST_ADDR1:
            s_parser.addr |= ((uint32_t)b << 8);
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            s_parser.state = ST_ADDR2;
            break;

        case ST_ADDR2:
            s_parser.addr |= ((uint32_t)b << 16);
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            s_parser.state = ST_ADDR3;
            break;

        case ST_ADDR3:
            s_parser.addr |= ((uint32_t)b << 24);
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            s_parser.state = (s_parser.len == 0) ? ST_CRC0 : ST_DATA;
            break;

        case ST_DATA:
            s_iap_rx_buf[s_parser.data_idx++] = b;
            s_parser.crc_calc = IAP_Crc16Modbus(s_parser.crc_calc, b);
            if (s_parser.data_idx >= s_parser.len) {
                s_parser.state = ST_CRC0;
            }
            break;

        case ST_CRC0:
            s_parser.crc_rx = b;
            s_parser.state = ST_CRC1;
            break;

        case ST_CRC1:
            s_parser.crc_rx |= ((uint16_t)b << 8);
            if (s_parser.crc_rx == s_parser.crc_calc) {
                s_frame_len = s_parser.len;
                s_frame_addr = s_parser.addr;
                s_frame_ready = true;
            }
            IAP_ParserReset();
            break;
    }
}

/* =========================================================================
 * Flash 操作
 * ========================================================================= */
static status_t IAP_FlashInit(void)
{
    return FLASH_Init(&s_flashDriver);
}

static status_t IAP_FlashFullEraseFrom(uint32_t appStart)
{
    uint32_t start = appStart - (appStart % IAP_PFLASH_SECTOR_SIZE);
    uint32_t end = IAP_PFLASH_BASE + IAP_PFLASH_SIZE;

    if (start >= end) {
        return kStatus_InvalidArgument;
    }

    uint32_t length = end - start;
    length = (length / IAP_PFLASH_SECTOR_SIZE) * IAP_PFLASH_SECTOR_SIZE;

    if (!length) {
        return kStatus_InvalidArgument;
    }

    return FLASH_Erase(&s_flashDriver, start, length, kFLASH_ApiEraseKey);
}

static status_t IAP_FlashProgramAligned(uint32_t addr, uint8_t *data, uint32_t len)
{
    const uint32_t align = IAP_FLASH_WRITE_UNIT;
    uint8_t buf[32];

    if (align > sizeof(buf)) {
        return kStatus_InvalidArgument;
    }

    status_t st;
    uint32_t cur = 0;

    while (cur < len) {
        uint32_t chunk = len - cur;
        if (chunk > align) {
            chunk = align;
        }

        if ((addr % align) != 0) {
            uint32_t padFront = addr % align;
            uint32_t alignedAdr = addr - padFront;

            for (uint32_t i = 0; i < align; i++) {
                buf[i] = 0xFF;
            }
            uint32_t copy = (chunk > (align - padFront)) ? (align - padFront) : chunk;
            memcpy(&buf[padFront], &data[cur], copy);

            st = FLASH_Program(&s_flashDriver, alignedAdr, buf, align);
            if (st != kStatus_Success) {
                return st;
            }

            addr += copy;
            cur += copy;
            continue;
        }

        if (chunk < align) {
            for (uint32_t i = 0; i < align; i++) {
                buf[i] = 0xFF;
            }
            memcpy(buf, &data[cur], chunk);
            st = FLASH_Program(&s_flashDriver, addr, buf, align);
            if (st != kStatus_Success) {
                return st;
            }
            addr += align;
            cur += chunk;
        } else {
            st = FLASH_Program(&s_flashDriver, addr, &data[cur], align);
            if (st != kStatus_Success) {
                return st;
            }
            addr += align;
            cur += align;
        }
    }
    return kStatus_Success;
}

/* =========================================================================
 * 其它 IAP 接口
 * ========================================================================= */
static inline void IAP_SetFlag(void)
{
    *((volatile uint32_t *)IAP_FLAG_ADDR) = IAP_MAGIC_VALUE;
}

static inline void IAP_ClrFlag(void)
{
    *((volatile uint32_t *)IAP_FLAG_ADDR) = 0u;
}

static inline bool IAP_IsFlagSet(void)
{
    // 修正：比较地址处的值，而不是地址本身
    return (*((volatile uint32_t *)IAP_FLAG_ADDR) == IAP_MAGIC_VALUE);
}

bool IAP_IsAppValid(uint32_t appAddr)
{
    uint32_t msp = *(uint32_t *)(appAddr + 0);
    uint32_t reset = *(uint32_t *)(appAddr + 4);

    // --- 新的、正确的栈指针检查 ---
    bool isMspInRam1 = (msp >= 0x1FFFE000u && msp <= 0x1FFFFFFFu);
    bool isMspInRam2 = (msp >= 0x20000000u && msp <= 0x20001FFFu);

    if (!isMspInRam1 && !isMspInRam2) {
        // 如果 MSP 不在任何一个有效的 RAM 区域内，则 App 无效
        return false;
    }
    // --- 检查结束 ---

    if (reset < appAddr || reset == 0xFFFFFFFFu) {
        return false;
    }
    
    return true;
}


void IAP_RequestUpdate(void)
{
    __disable_irq();
    IAP_SetFlag();
    __DSB();
    __ISB();
    NVIC_SystemReset();
}

bool IAP_CheckAndClearRequest(void)
{
    bool req = IAP_IsFlagSet();
    if (req) {
        IAP_ClrFlag();
    }
    return req;
}

static void IAP_WaitTxDone(void)
{
    while (!(UART1->S1 & kUART_TransmissionCompleteFlag)) {
    }
}

void IAP_JumpToApp(uint32_t appAddr)
{
    __disable_irq();
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    SCB->VTOR = appAddr;
    __DSB();
    __ISB();

    uint32_t appStack = *(uint32_t *)(appAddr + 0);
    uint32_t appEntry = *(uint32_t *)(appAddr + 4);

    __set_MSP(appStack);
    __DSB();
    __ISB();

    void (*app_reset_handler)(void) = (void (*)(void))appEntry;
    app_reset_handler();

    while (1) {
    }
}

void IAP_RunBootloader(void)
{
    myprintf("Update Mode(%s)\r\n", IAP_BL_VERSION_STR);

    /* 初始化 FLASH */
    __disable_irq();
    if (IAP_FlashInit() != kStatus_Success) {
        __enable_irq();
        myprintf("FLASH_INIT_ERR\r\n");
        while (1);
    }
    __enable_irq();

    /* 擦除 APP ~ Flash 尾 */
    __disable_irq();
    if (IAP_FlashFullEraseFrom(IAP_APP_START_ADDR) == kStatus_Success) {
        __enable_irq();
        myprintf("FLASH_ERASE_OK\r\n");
    } else {
        __enable_irq();
        myprintf("FLASH_ERASE_ERR\r\n");
        while (1);
    }

    /* 进入 IAP 解析模式 */
    IAP_ParserReset();
    g_iap_mode = true;

    /* 主循环：等待完整帧并处理 */
    while (1) {
        if (!s_frame_ready) {
            continue;
        }

        uint16_t length = s_frame_len;
        uint32_t addr = s_frame_addr;

        /* 清标志，准备接收下一帧 */
        s_frame_ready = false;

        /* 结束帧：length==0 且 addr==0xFFFFFFFF */
        if (length == 0u && addr == 0xFFFFFFFFu) {
            myprintf("DONE\r\n");
            IAP_WaitTxDone(); /* 确保最后一帧发送完毕 */
            IAP_JumpToApp(IAP_APP_START_ADDR);
            while (1);
        }

        /* 合法性检查 */
        if (addr < IAP_APP_START_ADDR || addr >= (IAP_PFLASH_BASE + IAP_PFLASH_SIZE)) {
            myprintf("ERR\r\n");
            continue;
        }

        /* 写入 Flash */
        __disable_irq();
        status_t st = IAP_FlashProgramAligned(addr, s_iap_rx_buf, length);
        __enable_irq();

        if (st == kStatus_Success) {
            myprintf("OK\r\n");
        } else {
            myprintf("ERR\r\n");
        }
    }
}
