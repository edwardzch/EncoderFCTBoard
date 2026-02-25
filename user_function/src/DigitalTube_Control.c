/****************************************************************************************
* @file      DigitalTube_Control.c
* @brief     数码管控制源文件 (动画/分页/编辑/保存完整逻辑 - 模块化版)
* @author    Gemini
* @date      2026-02-09
****************************************************************************************/
#include "DigitalTube_Control.h"
#include "Flash_Storage.h"
#include "Parameter_Module.h" // 引用参数模块
#include <string.h>

// 引用外部 SPI 句柄
extern SPI_HandleTypeDef hspi2;
// 引用外部报警变量
extern volatile uint8_t Work_Alarm;

// 全局变量定义
DTC_State_t DTC_Dev;

// 字库表 (共阳极段码)
// 索引: 0-15(0-F), 16(-), 17(Off), 18(H), 19(L), 20(P), 21(E), 22(_), 23(r), 24(t), 25(S)
const uint8_t DTC_SegTable[] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 
    0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E, 
    0xBF, 0xFF, 0x89, 0xC7, 0x8C, 0x86, 0xF7, 0xAF, 
    0x87, 0x92, 0xA3, 0xAB
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
#define SEG_o           26
#define SEG_n           27

// 【自定义】最高位分页符号 (0xFE = 上横杠)
#define SEG_HIGH_FLAG   0xFE 

// 位选码表
const uint8_t DTC_PosTable[] = {0x01, 0x02, 0x04, 0x08, 0x10};

// DMA 发送缓冲
static uint8_t DTC_DMA_Buffer[2];

// ================= 内部辅助函数 =================

/****************************************************************************************
* 函数名称：DTC_Update_Buffer
* 函数功能：根据当前模式和数据刷新显存
* 输入参量：无
* 输出参量：无
* 编写日期：2026-02-09
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
    // 使用 PM 模块获取配置
    PM_ParamConfig_t cfg = PM_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
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
        // Bin 格式下不需要光标位移 (或者固定位移?) 
        // 修正: 32位Bin显不下，假设只有低4位
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
* 函数功能：执行参数数值的加减运算以 (含Clamp逻辑)
* 输入参量：
* - is_up：1为加，0为减
* 输出参量：无
* 编写日期：2026-02-09
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
        // dP 参数组 (GroupIdx == 1) 为只读，不允许修改数值
        if (DTC_Dev.GroupIdx == 1) return;

        PM_ParamConfig_t cfg = PM_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
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

        // 极值限制与钳位逻辑 (Clamping)
        // 用户需求: 当千位调整导致数值越界时，自动变为边界值(例如 1234 -> 6234(X) -> 6000)
        // 且到达边界后不再继续增加/减少。
        if (temp > cfg.Max) temp = cfg.Max; 
        else if (temp < cfg.Min) temp = cfg.Min;
        
        DTC_Dev.EditVal = (int32_t)temp;
    }
    DTC_Update_Buffer();
}

/****************************************************************************************
* 函数名称：DTC_Key_Logic
* 函数功能：按键处理状态机 (含长短按复用、连发加速)
****************************************************************************************/
static void DTC_Key_Logic(void)
{
    uint32_t idr = DTC_KEY_PORT->IDR;
    
    // 读取原始按键状态 (0为按下)
    uint8_t k1_press = !(idr & PIN_MODE);
    uint8_t k2_press = !(idr & PIN_UP);
    uint8_t k3_press = !(idr & PIN_DOWN);
    uint8_t k4_press = !(idr & PIN_SHIFT);

    // --- 优先处理特殊组合键: 复位报错 (Key 2 + Key 3) ---
    if (k2_press && k3_press) {
        if (DTC_Dev.Mode == DTC_MODE_ERROR) {
            DTC_Dev.KeyTimer++;
            // 需要持续按下一小段时间防止误触 (比如50ms)
            if (DTC_Dev.KeyTimer >= 50 && !DTC_Dev.LongPressDone) {
                 DTC_Dev.LongPressDone = 1;
                 DTC_Dev.Mode = DTC_MODE_SELECT;
                 DTC_Dev.ErrCode = 0;
                 Work_Alarm = 0;
                 DTC_Update_Buffer();
            }
        }
        return; // 处理组合键时屏蔽其他单键
    }

    // --- 处理开机动画退出 (Key 1) ---
    if (DTC_Dev.Mode == DTC_MODE_ANIMATION && DTC_Dev.AnimState == ANIM_WAIT_KEY) {
        if (k1_press) {
            DTC_Dev.AnimState = ANIM_DONE;
            DTC_Dev.Mode = DTC_MODE_SELECT;
            
            // 关键修复: 抑制按键释放逻辑，防止首次进入Select模式时触发组切换
            DTC_Dev.LastKey = 1; 
            DTC_Dev.LongPressDone = 1; 
            
            DTC_Update_Buffer();
        }
        return; 
    }

    // --- 常规单键逻辑 (仅当无组合键时) ---
    uint8_t key_now = 0;
    if (k1_press) key_now = 1;
    else if (k2_press) key_now = 2;
    else if (k3_press) key_now = 3;
    else if (k4_press) key_now = 4;

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
                // 从 Parameter Module 加载数据到临时编辑变量
                DTC_Dev.EditVal = (DTC_Dev.GroupIdx == 0) ? PA_Buffer[DTC_Dev.ParamNum] : DP_Buffer[DTC_Dev.ParamNum];
                DTC_Dev.Page = PAGE_LOW; 
                
                // 暂存 Select 模式下的光标位置 (借用 ErrCode)
                DTC_Dev.ErrCode = DTC_Dev.EditBit; 
                DTC_Dev.EditBit = 0;     
            }
            else if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                // 长按：保存并退出
                if (DTC_Dev.GroupIdx == 0) PA_Buffer[DTC_Dev.ParamNum] = DTC_Dev.EditVal;
                else DP_Buffer[DTC_Dev.ParamNum] = DTC_Dev.EditVal;
                
                DTC_SaveParams_Callback(); // 触发外部保存
                
                // 恢复光标位置
                 DTC_Dev.EditBit = DTC_Dev.ErrCode; 
                 DTC_Dev.ErrCode = 0;
            }
            DTC_Update_Buffer();
        }

        // --- 连发逻辑 (Key 2/3) ---
        // 只有在非错误模式下才处理加减连发
        if (DTC_Dev.Mode != DTC_MODE_ERROR && DTC_Dev.KeyTimer >= KEY_LONG_MS && (key_now == 2 || key_now == 3)) {
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
                // 错误模式下屏蔽除复位外的单键操作
                if (DTC_Dev.Mode == DTC_MODE_ERROR) {
                     // 暂不响应单键，由组合键复位
                }
                else {
                    switch (DTC_Dev.LastKey) {
                        case 1: // Mod: 切换参数组 或 放弃编辑退出
                            if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                                DTC_Dev.Mode = DTC_MODE_SELECT; // 不保存，直接退
                                
                                // 恢复光标位置
                                DTC_Dev.EditBit = DTC_Dev.ErrCode; 
                                DTC_Dev.ErrCode = 0;
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
                                PM_ParamConfig_t cfg = PM_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
                                
                                // dP组只允许翻页，不允许移位编辑
                                if (DTC_Dev.GroupIdx == 1) {
                                    if (cfg.Format == FMT_DEC && cfg.Width == BIT_32) {
                                         // 32位允许切换分页查看
                                         if (++DTC_Dev.Page > PAGE_HIGH) DTC_Dev.Page = PAGE_LOW;
                                    }
                                }
                                else {
                                    // PA组: 允许翻页或移位
                                    if (cfg.Format == FMT_DEC && cfg.Width == BIT_32) {
                                        // 32位十进制：切换分页 (低->中->高)
                                        if (++DTC_Dev.Page > PAGE_HIGH) DTC_Dev.Page = PAGE_LOW;
                                    } else {
                                        // 其他格式：移位光标
                                        if (++DTC_Dev.EditBit > 3) DTC_Dev.EditBit = 0;
                                    }
                                }
                                DTC_Update_Buffer();
                            }
                            break;
                    }
                }
            }
            DTC_Dev.LastKey = 0; 
            DTC_Dev.KeyTimer = 0;
        }
    }
}

/****************************************************************************************
* 函数名称：DTC_HandleStartupAnimation
* 函数功能：执行开机动画
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
            
            if (DTC_Dev.AnimStep >= 5) { // 切换到等待模式
                DTC_Dev.AnimState = ANIM_WAIT_KEY;
                DTC_Dev.AnimStep = 0; 
                DTC_Dev.AnimTimer = 0;
            }
        }
    }
    // 阶段2：等待 Key1 确认退出 (由 Key_Logic 处理退出逻辑)
    else if (DTC_Dev.AnimState == ANIM_WAIT_KEY) {
        // 保持显示 Etest，什么都不需要做
    }
}

// ================= 外部调用接口 =================

/****************************************************************************************
* 函数名称：DTC_Init
* 函数功能：初始化硬件寄存器与软件状态
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
}

/****************************************************************************************
* 函数名称：DTC_ScanHandler
****************************************************************************************/
void DTC_ScanHandler(void)
{
    static uint8_t scan_idx = 0;
    static uint8_t last_alarm = 0;
    uint8_t char_code;

    // --- 0. 全局报警监测 (Work_Alarm) ---
    // 优先级最高: 只要有报警，强制切到错误模式
    if (Work_Alarm != last_alarm) {
        if (Work_Alarm != 0) {
            DTC_Dev.Mode = DTC_MODE_ERROR;
            DTC_Dev.ErrCode = Work_Alarm;
            DTC_Update_Buffer();
        } else {
             // 报警解除，恢复默认
             if (DTC_Dev.Mode == DTC_MODE_ERROR) {
                 DTC_Dev.Mode = DTC_MODE_SELECT;
                 DTC_Dev.ErrCode = 0;
                 DTC_Update_Buffer();
             }
        }
        last_alarm = Work_Alarm;
    }
    // 持续强制 (防止按键切出)
    if (Work_Alarm != 0 && DTC_Dev.Mode != DTC_MODE_ERROR) {
        DTC_Dev.Mode = DTC_MODE_ERROR;
        DTC_Dev.ErrCode = Work_Alarm;
        DTC_Update_Buffer();
    }

    // 1. 优先处理动画
    if (DTC_Dev.Mode == DTC_MODE_ANIMATION) {
        DTC_HandleStartupAnimation();
        // 允许在等待按键阶段扫描按键
        if (DTC_Dev.AnimState == ANIM_WAIT_KEY) {
             DTC_Key_Logic();
        }
    } else {
        DTC_Key_Logic(); 
    }

    // 2. 获取段码 (处理 0xFE 特殊符号)
    if (DTC_Dev.RawData[scan_idx] == SEG_HIGH_FLAG) {
        char_code = SEG_HIGH_FLAG; 
    } else {
        char_code = DTC_SegTable[DTC_Dev.RawData[scan_idx]];
    }
    
    // Err模式下 Err.20 固定点亮中间小数点
    if (DTC_Dev.Mode == DTC_MODE_ERROR && scan_idx == 2) char_code &= 0x7F;

    // 3. DP 闪烁光标逻辑 / 错误全局闪烁
    DTC_Dev.BlinkCnt++;
    if (DTC_Dev.BlinkCnt >= 400) DTC_Dev.BlinkCnt = 0;
    
    // --- 处理消息模式计时 ---
    if (DTC_Dev.Mode == DTC_MODE_MESSAGE) {
        DTC_Dev.MsgTimer++;
        if (DTC_Dev.MsgTimer >= 1200) { // 300ms * 4 = 1.2s (闪烁2次)
            DTC_Dev.MsgTimer = 0;
            DTC_Dev.Mode = DTC_MODE_SELECT; // 退出编辑
            DTC_Update_Buffer();
        }
        
        // 闪烁逻辑: 300ms 灭, 300ms 亮
        if ((DTC_Dev.MsgTimer / 300) % 2 == 0) {
            char_code = DTC_SegTable[SEG_OFF];
        }
    }

    // 只有在非动画模式下才处理
    if (DTC_Dev.Mode != DTC_MODE_ANIMATION) {
       
        // ---- A. 故障报错整屏闪烁 ----
        if (DTC_Dev.Mode == DTC_MODE_ERROR) {
             if (DTC_Dev.BlinkCnt >= 200) {
                 char_code = DTC_SegTable[SEG_OFF]; 
             }
        }
        else {
             // ---- B. 光标位闪烁 ----
            uint8_t blink_pos = 0xFF; 

            // 情况A: 选择界面 (PA 001)
            if (DTC_Dev.Mode == DTC_MODE_SELECT) {
                blink_pos = DTC_Dev.EditBit; 
            } 
            // 情况B: 编辑界面 (数值)
            else if (DTC_Dev.Mode == DTC_MODE_EDIT) {
                // 如果是 dP 组，强制不闪烁 (只读)
                if (DTC_Dev.GroupIdx == 1) {
                    blink_pos = 0xFF;
                }
                else {
                    PM_ParamConfig_t cfg = PM_GetConfig(DTC_Dev.GroupIdx, DTC_Dev.ParamNum);
                    // 32位分页模式通常不闪烁位(因为在翻页)，其他格式闪烁编辑位
                    if (!(cfg.Format == FMT_DEC && cfg.Width == BIT_32)) {
                        blink_pos = DTC_Dev.EditBit;
                    }
                }
            }

            // 执行闪烁
            if (scan_idx == blink_pos && DTC_Dev.BlinkCnt < 200) {
                char_code &= 0x7F; // 点亮 DP
            }
        }
    }

    // 4. DMA 发送
    DTC_DMA_Transmitter(char_code, DTC_PosTable[scan_idx]);
    
    if (++scan_idx >= 5) scan_idx = 0;
}

/****************************************************************************************
* 函数名称：DTC_SetError
****************************************************************************************/
void DTC_SetError(uint16_t code) 
{ 
    DTC_Dev.ErrCode = code; 
    DTC_Dev.Mode = DTC_MODE_ERROR; 
    DTC_Update_Buffer(); 
}

/****************************************************************************************
* 函数名称：DTC_SaveParams_Callback
****************************************************************************************/
void DTC_SaveParams_Callback(void) 
{
    // 1. 先切换到消息提示模式
    DTC_Dev.Mode = DTC_MODE_MESSAGE;
    DTC_Dev.MsgTimer = 0;
    
    // 确保 Flash 写入期间数码管全灭
    DTC_RCLK_L(); 
    uint8_t temp_buf[2] = {0, 0xFF}; // Pos=0, Seg=OFF
    HAL_SPI_Transmit(&hspi2, temp_buf, 2, 10);
    DTC_RCLK_H();
    
    HAL_Delay(5); 
    
    // 2. 保存参数到 Flash (调用 PM 模块接口)
    PM_SaveParams();
}
