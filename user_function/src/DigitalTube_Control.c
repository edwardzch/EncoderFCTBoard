/****************************************************************************************
* @file      DigitalTube_Control.c
* @brief     数码管控制源文件 (动画/分页/编辑/保存完整逻辑)
* @author    Gemini
* @date      2026-02-06
****************************************************************************************/
#include "DigitalTube_Control.h"
#include <string.h>

// 引用外部 SPI 句柄
extern SPI_HandleTypeDef hspi2;

// 全局变量定义
DTC_State_t DTC_Dev;
int32_t PA_Buffer[PA_SIZE];
int32_t DP_Buffer[DP_SIZE];

// 字库表 (共阳极段码)
// 索引: 0-15(0-F), 16(-), 17(Off), 18(H), 19(L), 20(P), 21(E), 22(_), 23(r), 24(t), 25(S)
const uint8_t DTC_SegTable[] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 
    0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E, 
    0xBF, 0xFF, 0x89, 0xC7, 0x8C, 0x86, 0xF7, 0xAF, 
    0x87, 0x92 
};

// 字库索引宏定义
#define SEG_P           20
#define SEG_A           10
#define SEG_d           13
#define SEG_E           21  
#define SEG_r           23
#define SEG_UNDER       22
#define SEG_MINUS       16
#define SEG_OFF         17
#define SEG_H           18
#define SEG_b           11
#define SEG_t           24
#define SEG_S           25

// 【自定义】最高位分页符号 (0xFE = 上横杠)
// 注意：此值直接作为段码发送，不查 SegTable
#define SEG_HIGH_FLAG   0xFE 

// 位选码表
const uint8_t DTC_PosTable[] = {0x01, 0x02, 0x04, 0x08, 0x10};

// DMA 发送缓冲
static uint8_t DTC_DMA_Buffer[2];

// ================= 内部辅助函数 =================

/****************************************************************************************
* 函数名称：DTC_GetConfig
* 函数功能：模拟获取每个参数的属性配置 (实际可替换为查表)
* 输入参量：
* - group：参数组索引
* - index：参数编号
* 输出参量：
* - DTC_ParamConfig_t：参数属性结构体
* 编写日期：2026-02-06
****************************************************************************************/
static DTC_ParamConfig_t DTC_GetConfig(uint8_t group, uint16_t index)
{
    DTC_ParamConfig_t cfg;
    // 默认配置: 16位有符号十进制
    cfg.Sign = SIGNED;
    cfg.Format = FMT_DEC;
    cfg.Width = BIT_16;
    cfg.Min = -9999;
    cfg.Max = 9999;

    // 自定义特殊参数示例
    if (group == 0 && index == 0) { // PA000: 32位大数
        cfg.Width = BIT_32; 
        cfg.Min = -2000000000; 
        cfg.Max = 2000000000; 
    }
    if (group == 0 && index == 1) { // PA001: 16进制
        cfg.Format = FMT_HEX; 
        cfg.Min = 0; 
        cfg.Max = 0xFFFF;
    }
    if (group == 1 && index == 0) { // DP000: 2进制
        cfg.Format = FMT_BIN; 
        cfg.Min = 0; 
        cfg.Max = 0xF;
    }
    return cfg;
}

/****************************************************************************************
* 函数名称：DTC_Update_Buffer
* 函数功能：根据当前模式和数据刷新显存
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
static void DTC_Update_Buffer(void)
{
    // 动画模式下不由该函数控制
    if (DTC_Dev.Mode == DTC_MODE_ANIMATION) return;
    
    memset(DTC_Dev.RawData, SEG_OFF, 5); 

    // --- 1. 错误显示 Err.20 ---
    if (DTC_Dev.Mode == DTC_MODE_ERROR) {
        DTC_Dev.RawData[4] = SEG_E; 
        DTC_Dev.RawData[3] = SEG_r; 
        DTC_Dev.RawData[2] = SEG_r;
        DTC_Dev.RawData[1] = (DTC_Dev.ErrCode / 10) % 10; 
        DTC_Dev.RawData[0] = DTC_Dev.ErrCode % 10;
        return;
    }

    // --- 2. 选择界面 (PA 001) ---
    if (DTC_Dev.Mode == DTC_MODE_SELECT) {
        DTC_Dev.RawData[4] = (DTC_Dev.GroupIdx == 0) ? SEG_P : SEG_d;
        DTC_Dev.RawData[3] = (DTC_Dev.GroupIdx == 0) ? SEG_A : SEG_P;
        DTC_Dev.RawData[2] = (DTC_Dev.ParamNum / 100) % 10;
        DTC_Dev.RawData[1] = (DTC_Dev.ParamNum / 10) % 10;
        DTC_Dev.RawData[0] = DTC_Dev.ParamNum % 10;
        return;
    }

    // --- 3. 编辑/查看数值模式 ---
    DTC_ParamConfig_t cfg = DTC_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
    int32_t val = DTC_Dev.EditVal;

    // A. HEX 格式 (H.xxxx)
    if (cfg.Format == FMT_HEX) {
        DTC_Dev.RawData[4] = SEG_H;
        for(int i=0; i<4; i++) { 
            DTC_Dev.RawData[i] = (val >> (i * 4)) & 0xF; 
        }
    }
    // B. BIN 格式 (b.xxxx)
    else if (cfg.Format == FMT_BIN) {
        DTC_Dev.RawData[4] = SEG_b;
        for(int i=0; i<4; i++) { 
            DTC_Dev.RawData[i] = (val >> i) & 1; 
        }
    }
    // C. DEC 格式 (含分页)
    else {
        uint32_t abs_val = (val < 0) ? -val : val;
        
        // 16位数据: 不分页
        if (cfg.Width == BIT_16) {
            DTC_Dev.RawData[4] = (val < 0) ? SEG_MINUS : SEG_OFF;
            for(int i=0; i<4; i++) { 
                DTC_Dev.RawData[i] = abs_val % 10; 
                abs_val /= 10; 
            }
        }
        // 32位数据: 分页显示
        else {
            if (DTC_Dev.Page == PAGE_LOW) { // 低位: _ 1234
                DTC_Dev.RawData[4] = SEG_UNDER;
                for(int i=0; i<4; i++) { 
                    DTC_Dev.RawData[i] = abs_val % 10; 
                    abs_val /= 10; 
                }
            }
            else if (DTC_Dev.Page == PAGE_MID) { // 中位: - 5678
                DTC_Dev.RawData[4] = SEG_MINUS;
                abs_val /= 10000;
                for(int i=0; i<4; i++) { 
                    DTC_Dev.RawData[i] = abs_val % 10; 
                    abs_val /= 10; 
                }
            }
            else { // 高位: [0xFE]  90
                DTC_Dev.RawData[4] = SEG_HIGH_FLAG; // 显示特殊顶杠符号
                abs_val /= 100000000;
                if (abs_val > 0) {
                    DTC_Dev.RawData[0] = abs_val % 10;
                    if (abs_val >= 10) DTC_Dev.RawData[1] = (abs_val / 10) % 10;
                } else {
                    DTC_Dev.RawData[0] = 0; // 若无高位，至少显示0
                }
            }
        }
    }
}

/****************************************************************************************
* 函数名称：DTC_DMA_Transmitter
* 函数功能：底层 DMA 传输 (阻塞式)
* 输入参量：
* - seg：段码数据
* - pos：位选数据
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
static void DTC_DMA_Transmitter(uint8_t seg, uint8_t pos)
{
    DTC_DMA_Buffer[0] = pos; 
    DTC_DMA_Buffer[1] = seg;
    
    DTC_RCLK_L(); 
    
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;  
    DMA1->IFCR = 0x0F;                  // 清除所有中断标志
    DMA1_Channel1->CNDTR = 2;           
    DMA1_Channel1->CCR |= DMA_CCR_EN;   
    
    // 等待 DMA 传输完成
    while(!(DMA1->ISR & DMA_ISR_TCIF1)); 
    // 等待 SPI FIFO 清空
    while((SPI2->SR & SPI_SR_FTLVL) != 0);
    // 等待 SPI 总线空闲
    while(SPI2->SR & SPI_SR_BSY);
    
    // 短延时确保锁存稳定
    for(volatile uint8_t i=0; i<15; i++);
    
    DTC_RCLK_H();   
}

/****************************************************************************************
* 函数名称：DTC_Apply_Edit
* 函数功能：执行参数数值的加减运算
* 输入参量：
* - is_up：1为加，0为减
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
static void DTC_Apply_Edit(uint8_t is_up)
{
    // A. 在选择界面：修改参数编号
    if (DTC_Dev.Mode == DTC_MODE_SELECT) {
        int32_t step = 1;
        // 根据光标位置计算步进 (0:个位, 1:十位, 2:百位)
        for(uint8_t i=0; i<DTC_Dev.EditBit; i++) step *= 10;
        
        int32_t max_idx = (DTC_Dev.GroupIdx == 0) ? PA_SIZE : DP_SIZE;
        int32_t new_idx = DTC_Dev.ParamNum;
        
        if (is_up) new_idx += step; else new_idx -= step;
        
        // 循环限制
        while (new_idx >= max_idx) new_idx -= max_idx;
        while (new_idx < 0) new_idx += max_idx;
        
        DTC_Dev.ParamNum = (uint16_t)new_idx;
    }
    // B. 在编辑界面：修改数值内容
    else {
        DTC_ParamConfig_t cfg = DTC_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
        int64_t step = 1; 
        
        if (cfg.Format == FMT_DEC) {
            int power = DTC_Dev.EditBit; 
            // 32位数据需叠加分页权重
            if (cfg.Width == BIT_32) {
                if (DTC_Dev.Page == PAGE_MID) power += 4;
                if (DTC_Dev.Page == PAGE_HIGH) power += 8;
            }
            for(int i=0; i<power; i++) step *= 10;
        } 
        else if (cfg.Format == FMT_HEX) { 
            for(int i=0; i<DTC_Dev.EditBit; i++) step *= 16; 
        }
        else if (cfg.Format == FMT_BIN) { 
            for(int i=0; i<DTC_Dev.EditBit; i++) step *= 2; 
        }

        int64_t temp = DTC_Dev.EditVal;
        if (is_up) temp += step; else temp -= step;

        // 极值限制
        if (temp > cfg.Max) temp = cfg.Min; 
        else if (temp < cfg.Min) temp = cfg.Max;
        
        DTC_Dev.EditVal = (int32_t)temp;
    }
    DTC_Update_Buffer();
}

/****************************************************************************************
* 函数名称：DTC_Key_Logic
* 函数功能：按键处理状态机 (含长短按复用、连发加速)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
static void DTC_Key_Logic(void)
{
    uint8_t key_now = 0; 
    uint32_t idr = DTC_KEY_PORT->IDR;

    if (!(idr & PIN_MODE))       key_now = 1;
    else if (!(idr & PIN_UP))    key_now = 2;
    else if (!(idr & PIN_DOWN))  key_now = 3;
    else if (!(idr & PIN_SHIFT)) key_now = 4;

    if (key_now != 0) {
        // --- 按键按下时刻 ---
        if (key_now != DTC_Dev.LastKey) {
            DTC_Dev.KeyTimer = 0; 
            DTC_Dev.LongPressDone = 0; 
            DTC_Dev.LastKey = key_now;
            DTC_Dev.RepeatTimer = 0; 
            DTC_Dev.CurrentSpeed = ACCEL_START_MS;
        }
        DTC_Dev.KeyTimer++;

        // --- 长按检测 (仅 Key 4) ---
        // 只有 Key 4 支持长按切换模式/保存
        if (key_now == 4 && DTC_Dev.KeyTimer >= KEY_LONG_MS && !DTC_Dev.LongPressDone) {
            DTC_Dev.LongPressDone = 1; // 标记长按已处理
            
            if (DTC_Dev.Mode == DTC_MODE_SELECT) {
                // 长按：进入编辑模式
                DTC_Dev.Mode = DTC_MODE_EDIT;
                // 从Buffer加载数据到临时编辑变量
                DTC_Dev.EditVal = (DTC_Dev.GroupIdx == 0) ? PA_Buffer[DTC_Dev.ParamNum] : DP_Buffer[DTC_Dev.ParamNum];
                DTC_Dev.Page = PAGE_LOW; 
                DTC_Dev.EditBit = 0;     
            }
            else if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                // 长按：保存并退出
                if (DTC_Dev.GroupIdx == 0) PA_Buffer[DTC_Dev.ParamNum] = DTC_Dev.EditVal;
                else DP_Buffer[DTC_Dev.ParamNum] = DTC_Dev.EditVal;
                
                DTC_SaveParams_Callback(); // 触发外部保存
                DTC_Dev.Mode = DTC_MODE_SELECT;
            }
            DTC_Update_Buffer();
        }

        // --- 连发逻辑 (Key 2/3) ---
        if (DTC_Dev.KeyTimer >= KEY_LONG_MS && (key_now == 2 || key_now == 3)) {
            DTC_Dev.RepeatTimer++;
            if (DTC_Dev.RepeatTimer >= DTC_Dev.CurrentSpeed) {
                DTC_Dev.RepeatTimer = 0;
                DTC_Apply_Edit(key_now == 2 ? 1 : 0);
                // 平滑加速
                if (DTC_Dev.CurrentSpeed > ACCEL_MIN_MS) DTC_Dev.CurrentSpeed -= ACCEL_STEP;
            }
        }
    } 
    else { 
        // --- 按键释放时刻 (短按触发点) ---
        if (DTC_Dev.LastKey != 0) {
            // 如果未触发过长按，且时间超过消抖，则判定为短按
            if (!DTC_Dev.LongPressDone && DTC_Dev.KeyTimer >= KEY_DEBOUNCE_MS) {
                switch (DTC_Dev.LastKey) {
                    case 1: // Mod: 切换参数组 或 放弃编辑退出
                        if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                            DTC_Dev.Mode = DTC_MODE_SELECT; // 不保存，直接退
                        } else {
                            DTC_Dev.GroupIdx = !DTC_Dev.GroupIdx;
                            DTC_Dev.ParamNum = 0;
                        }
                        DTC_Update_Buffer();
                        break;
                    case 2: DTC_Apply_Edit(1); break; // Up: 加
                    case 3: DTC_Apply_Edit(0); break; // Down: 减
                    case 4: // Shift: 短按
                        if (DTC_Dev.Mode == DTC_MODE_SELECT) {
                            // 选择界面：左移光标 (个->十->百)
                            if (++DTC_Dev.EditBit > 2) DTC_Dev.EditBit = 0;
                        } 
                        else if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                            // 编辑界面：
                            DTC_ParamConfig_t cfg = DTC_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
                            if (cfg.Format == FMT_DEC && cfg.Width == BIT_32) {
                                // 32位十进制：切换分页 (低->中->高)
                                if (++DTC_Dev.Page > PAGE_HIGH) DTC_Dev.Page = PAGE_LOW;
                            } else {
                                // 其他格式：移位光标
                                if (++DTC_Dev.EditBit > 3) DTC_Dev.EditBit = 0;
                            }
                            DTC_Update_Buffer();
                        }
                        break;
                }
            }
            DTC_Dev.LastKey = 0; 
            DTC_Dev.KeyTimer = 0;
        }
    }
}

/****************************************************************************************
* 函数名称：DTC_HandleStartupAnimation
* 函数功能：执行开机动画 (打字机效果 + 闪烁)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
static void DTC_HandleStartupAnimation(void)
{
    // 阶段1：打字机 (E -> Et -> Ete -> Etes -> Etest)
    if (DTC_Dev.AnimState == ANIM_TYPEWRITER) {
        DTC_Dev.AnimTimer++;
        if (DTC_Dev.AnimTimer >= 150) { // 150ms 间隔
            DTC_Dev.AnimTimer = 0;
            DTC_Dev.AnimStep++;
            
            memset(DTC_Dev.RawData, SEG_OFF, 5);
            // 倒序填充缓冲区
            if (DTC_Dev.AnimStep >= 1) DTC_Dev.RawData[4] = SEG_E;
            if (DTC_Dev.AnimStep >= 2) DTC_Dev.RawData[3] = SEG_t;
            if (DTC_Dev.AnimStep >= 3) DTC_Dev.RawData[2] = SEG_E;
            if (DTC_Dev.AnimStep >= 4) DTC_Dev.RawData[1] = SEG_S;
            if (DTC_Dev.AnimStep >= 5) DTC_Dev.RawData[0] = SEG_t;
            
            if (DTC_Dev.AnimStep >= 5) { // 切换到闪烁阶段
                DTC_Dev.AnimState = ANIM_BLINK;
                DTC_Dev.AnimStep = 0; 
                DTC_Dev.AnimTimer = 0;
            }
        }
    }
    // 阶段2：整体闪烁3次
    else if (DTC_Dev.AnimState == ANIM_BLINK) {
        DTC_Dev.AnimTimer++;
        if (DTC_Dev.AnimTimer >= 300) { // 300ms 翻转
            DTC_Dev.AnimTimer = 0;
            DTC_Dev.AnimStep++;
            
            if (DTC_Dev.AnimStep > 6) { // 6次翻转 = 3次闪烁
                DTC_Dev.AnimState = ANIM_DONE;
                DTC_Dev.Mode = DTC_MODE_SELECT; // 进入主界面
                DTC_Update_Buffer();
            } else {
                if (DTC_Dev.AnimStep % 2 != 0) {
                    memset(DTC_Dev.RawData, SEG_OFF, 5); // 灭
                } else {
                    // 亮 Etest
                    DTC_Dev.RawData[4]=SEG_E; DTC_Dev.RawData[3]=SEG_t; DTC_Dev.RawData[2]=SEG_E; DTC_Dev.RawData[1]=SEG_S; DTC_Dev.RawData[0]=SEG_t;
                }
            }
        }
    }
}

// ================= 外部调用接口 =================

/****************************************************************************************
* 函数名称：DTC_Init
* 函数功能：初始化硬件寄存器与软件状态
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
void DTC_Init(void)
{
    memset(&DTC_Dev, 0, sizeof(DTC_Dev));
    
    // 初始化 SPI 与 DMA
    SPI2->CR2 |= (SPI_CR2_FRXTH | SPI_CR2_TXDMAEN); 
    DMA1_Channel1->CPAR = (uint32_t)&SPI2->DR;
    DMA1_Channel1->CMAR = (uint32_t)DTC_DMA_Buffer;
    SPI2->CR1 |= SPI_CR1_SPE;       

    // 设置初始模式为开机动画
    DTC_Dev.Mode = DTC_MODE_ANIMATION;
    DTC_Dev.AnimState = ANIM_TYPEWRITER;
    DTC_Dev.AnimStep = 0;
    DTC_Dev.AnimTimer = 0;
    
    // 初始化测试数据
    PA_Buffer[0] = 1234567890; // 测试32位大数
    PA_Buffer[1] = 0xABCD;     // 测试Hex
}

/****************************************************************************************
* 函数名称：DTC_ScanHandler
* 函数功能：定时扫描处理函数 (需在 1ms 定时器中断中调用)
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
void DTC_ScanHandler(void)
{
    static uint8_t scan_idx = 0;
    uint8_t char_code;

    // 1. 优先处理动画
    if (DTC_Dev.Mode == DTC_MODE_ANIMATION) {
        DTC_HandleStartupAnimation();
    } else {
        DTC_Key_Logic(); 
    }

    // 2. 获取段码 (处理 0xFE 特殊符号)
    if (DTC_Dev.RawData[scan_idx] == SEG_HIGH_FLAG) {
        char_code = SEG_HIGH_FLAG; 
    } else {
        char_code = DTC_SegTable[DTC_Dev.RawData[scan_idx]];
    }
    
    // 3. DP 闪烁光标逻辑 (核心修改部分)
    DTC_Dev.BlinkCnt++;
    if (DTC_Dev.BlinkCnt >= 400) DTC_Dev.BlinkCnt = 0;

    // 只有在非动画模式下才处理闪烁
    if (DTC_Dev.Mode != DTC_MODE_ANIMATION) {
        uint8_t blink_pos = 0xFF; // 0xFF表示不闪烁

        // 情况A: 选择界面 (PA 001)
        // EditBit 0->个位(RawData[0]), 1->十位(RawData[1]), 2->百位(RawData[2])
        if (DTC_Dev.Mode == DTC_MODE_SELECT) {
            blink_pos = DTC_Dev.EditBit; 
        } 
        // 情况B: 编辑界面 (数值)
        else if (DTC_Dev.Mode == DTC_MODE_EDIT) {
            DTC_ParamConfig_t cfg = DTC_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
            // 32位分页模式通常不闪烁位(因为在翻页)，其他格式闪烁编辑位
            if (!(cfg.Format == FMT_DEC && cfg.Width == BIT_32)) {
                blink_pos = DTC_Dev.EditBit;
            }
        }

        // 执行闪烁: 周期前200ms点亮DP
        if (scan_idx == blink_pos && DTC_Dev.BlinkCnt < 200) {
            char_code &= 0x7F; // 点亮 DP
        }
    }
    
    // Err模式下 Err.20 固定点亮中间小数点
    if (DTC_Dev.Mode == DTC_MODE_ERROR && scan_idx == 2) char_code &= 0x7F;

    // 4. DMA 发送
    DTC_DMA_Transmitter(char_code, DTC_PosTable[scan_idx]);
    
    if (++scan_idx >= 5) scan_idx = 0;
}

/****************************************************************************************
* 函数名称：DTC_SetError
* 函数功能：进入故障显示模式
* 输入参量：code - 错误代码
* 输出参量：无
* 编写日期：2026-02-06
****************************************************************************************/
void DTC_SetError(uint16_t code) 
{ 
    DTC_Dev.ErrCode = code; 
    DTC_Dev.Mode = DTC_MODE_ERROR; 
    DTC_Update_Buffer(); 
}

// 弱定义回调函数
__weak void DTC_SaveParams_Callback(void) {}
	
	