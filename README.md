# EncoderFCTBoard

基于 **STM32G491CCU3** 的编码器 FCT（功能测试）板固件，用于对多种工业编码器进行通讯测试、参数读写、EEPROM 读写、转速计算等功能验证。

---

## 硬件概览

| 项目         | 说明                                     |
| ------------ | ---------------------------------------- |
| MCU          | STM32G491CCU3 (ARM Cortex-M4, 170MHz)    |
| Flash        | 256KB (末尾 2 页用于参数存储)              |
| 编码器通讯    | USART3 (RS485 半双工, DMA TX/RX)          |
| Modbus 通讯  | USART1 (RS485 从机, DMA)                  |
| 数码管显示    | SPI2 → 74AHC595D 移位寄存器 × 5 位        |
| 按键输入      | 4 键 (KEY1 功能, KEY2 加, KEY3 减, KEY4 移位) |
| 继电器输出    | 8 路继电器控制                             |
| Bootloader   | IAP 支持, 应用起始地址 0x08005000          |

---

## 目录结构

```
EncoderFCTBoard/
├── Core/                       # STM32CubeMX 生成的 HAL 驱动
│   ├── Inc/                    #   头文件 (main.h, gpio.h, dma.h, usart.h, tim.h, spi.h)
│   └── Src/                    #   源文件
├── Drivers/                    # STM32 HAL 库
├── MDK-ARM/                    # Keil MDK 工程文件
├── user_config/                # 用户配置
│   ├── inc/                    #   gpio_config.h, uart_config.h
│   └── src/
├── user_function/              # ★ 主要业务代码
│   ├── inc/                    #   所有模块头文件
│   └── src/                    #   所有模块源文件
├── EncoderTest_V2.2/           # 旧版参考工程
├── EncoderFCTBoard.ioc         # CubeMX 工程文件
└── README.md                   # 本文件
```

---

## 模块说明

### 1. 编码器底层驱动 (`encoder_driver`)

| 文件 | 说明 |
| ---- | ---- |
| `encoder_driver.h/c` | USART3 + DMA 底层收发驱动 |

- RS485 半双工控制 (DE/RE 引脚 PB3)
- DMA 非阻塞发送 + 接收，发送完成 TC 中断自动切换收发方向
- 支持动态修改波特率、通信周期
- TIM1 定时器驱动周期性通信调度

### 2. 编码器调度器 (`encoder_dispatcher`)

| 文件 | 说明 |
| ---- | ---- |
| `encoder_dispatcher.h/c` | 编码器测试调度 + RX 数据分发 |

- TIM1 周期中断中调用 `Encoder_DispatchTest()`，根据编码器类型分发测试命令
- DMA 接收完成后调用 `Encoder_DispatchRx()`，将数据拷贝到对应编码器模块并触发解析
- 提供 `Encoder_GetSingleTurnPos()` 和 `Encoder_GetResolutionBits()` 通用接口

### 3. 编码器协议模块 (6 种)

| 模块 | 文件 | Modbus 地址段 | 说明 |
| ---- | ---- | ------------- | ---- |
| 多圈磁编码器 | `Encoder_MultiturnMag.h/c` | `0x0200-0x02FF` | 完整测试 (通讯/EEPROM/批量读写/初始化/复位/分辨率设置) |
| 多圈光编码器 | `Encoder_MultiturnOpt.h/c` | `0x0300-0x03FF` | 通讯/EEPROM 批量读写/初始化 |
| MGT 磁编码器 | `Encoder_MGTMag.h/c`       | `0x0400-0x04FF` | 通讯/EEPROM 批量读写/初始化 (ced/CIP/OIP) |
| NXP 磁编码器 | `Encoder_NXPMag.h/c`       | `0x0500-0x05FF` | 通讯/EEPROM/固件版本读取 |
| 多摩川编码器 | `Encoder_Tamagawa.h/c`     | `0x0600-0x06FF` | 通讯/EEPROM/MTP 读写/多圈复位/单圈复位 |
| SensAR 编码器 | `Encoder_SensAR.h/c`       | `0x0700-0x07FF` | ASCII 协议，Hall 值读取/产品信息/版本号 |

**每个模块统一实现的接口：**
- `Xxx_Init()` — 初始化
- `Xxx_TX()` — 发送命令（switch-case 风格，含超时检测）
- `Xxx_RxHandler()` — 逐字节接收处理
- `Xxx_RxComplete()` — 接收完成解析
- `Xxx_Test()` — 测试状态机主入口
- `Xxx_Modbus_IsMyAddr()` — Modbus 地址归属判断
- `Xxx_Modbus_Read()` — Modbus 03H 读取
- `Xxx_Modbus_Write()` — Modbus 06H 写入

### 4. Modbus 通讯 (`modbus_function` + `encoder_modbus`)

| 文件 | 说明 |
| ---- | ---- |
| `modbus_function.h/c` | Modbus RTU 从机协议栈 (USART1, 03H/04H/06H/10H) |
| `encoder_modbus.h/c`  | 编码器寄存器路由层，Modbus 读写请求分发到各编码器模块 |

**Modbus 寄存器地址分区：**

| 地址范围 | 说明 |
| -------- | ---- |
| `0x0000-0x00FF` | 基础配置区 (编码器类型、波特率、通讯周期、报警等) |
| `0x0200-0x02FF` | 多圈磁编码器 |
| `0x0300-0x03FF` | 多圈光编码器 |
| `0x0400-0x04FF` | MGT 磁编码器 |
| `0x0500-0x05FF` | NXP 磁编码器 |
| `0x0600-0x06FF` | 多摩川编码器 |
| `0x0700-0x07FF` | SensAR 编码器 |
| `0x1000-0x1039` | 通用寄存器区 (modbus_function 管理) |

### 5. 转速计算 (`encoder_speed`)

| 文件 | 说明 |
| ---- | ---- |
| `encoder_speed.h/c` | 位置差分 + 中位值滤波转速计算 |

- 支持动态分辨率和采样周期更新
- 5 点中位值滤波器消除毛刺

### 6. 数码管与按键 (`DigitalTube_Control`)

| 文件 | 说明 |
| ---- | ---- |
| `DigitalTube_Control.h/c` | 5 位数码管显示 + 4 键输入状态机 |

**显示模式：**
- `ANIMATION` — 开机打字机动画 ("Etest")
- `SELECT` — 参数组/编号选择 (PA/dP + 编号)
- `EDIT` — 参数值编辑 (支持十进制/十六进制/二进制，16/32 位翻页)
- `ERROR` — 故障报错 (显示 Err + 错误码)
- `MESSAGE` — 消息提示 (donE/SAVE 闪烁)

**按键功能：**
- KEY1 (功能) — 切换参数组 / 退出编辑
- KEY2 (加) — 值递增 / 长按连发加速
- KEY3 (减) — 值递减 / 长按连发加速
- KEY4 (移位) — 短按翻页/移位，长按进入编辑/保存
- KEY2+KEY3 (组合) — 错误模式下复位报警

### 7. 参数管理 (`Parameter_Module`)

| 文件 | 说明 |
| ---- | ---- |
| `Parameter_Module.h/c` | PA/dP 参数组管理 (各 60 个参数) |

- 每个参数配置：符号(有符号/无符号)、显示进制(DEC/HEX/BIN)、位宽(16/32位)、范围限制

**PA 参数表 (可读写, Flash 持久化)：**

| 参数 | 功能 | 格式 | 范围 | 默认值 | 说明 |
|------|------|------|------|--------|------|
| PA000 | Modbus 从机地址 | DEC | 1~255 | 5 | |
| PA001 | 波特率 / 100 | DEC | 12~1152 | 576 | 576=57600 |
| PA002 | 校验位 | 文字 | 0~2 | 1(odd) | 显示 nonE / odd / EuEn |
| PA003 | K1-K4 继电器 | BIN | b.0000~b.1111 | 0 | bit0=K1, bit3=K4 |
| PA004 | K5-K8 继电器 | BIN | b.0000~b.1111 | 0 | bit0=K5, bit3=K8 |
| PA005 | 恢复出厂设置 | DEC | 0~9999 | 0 | 输入 1234 触发恢复 |
| PA010 | ReadReg 地址 | HEX | 0x0000~0xFFFF | 0 | 编码器寄存器读地址 |
| PA011 | WriteReg 地址 | HEX | 0x0000~0xFFFF | 0 | 编码器寄存器写地址 |
| PA012 | WriteReg 数据 | HEX | 0x0000~0xFFFF | 0 | 保存后立即写入 |

**DP 参数表 (只读, 运行时更新)：**

| 参数 | 功能 | 格式 | 说明 |
|------|------|------|------|
| DP000 | 固件版本 | DEC | 1.0 → 显示 10 |
| DP001 | 继电器状态 | BIN | K1-K4 / K5-K8 分页, UP/DOWN 切换 |
| DP010 | ReadReg 返回值 | HEX | 进入时自动读取 PA010 地址 |

### 8. Flash 存储 (`Flash_Storage`)

| 文件 | 说明 |
| ---- | ---- |
| `Flash_Storage.h/c` | Append-Only Flash 参数持久化 |

- 使用 Flash 末尾 2 页 (Page 126 + 127) 双页交替存储
- 每条记录 256 字节 (Header 8B + Data 240B + Footer 8B)，每页 8 条
- CRC32 校验，Magic 标识有效记录，Index 递增序列号
- 加载返回值：0=成功, 1=Flash 为空(Err.10), 2=CRC 失败(Err.11)

### 9. 继电器控制 (`relay_control`)

| 文件 | 说明 |
| ---- | ---- |
| `relay_control.h/c` | 8 路继电器开关控制 |

- 支持单路控制 (On/Off/Toggle)、掩码控制 (SetByMask)、逐个测试 (Relay_Test)
- Modbus 地址 `0x1000`：按位掩码控制 K1-K8
- Modbus 地址 `0x1009`：继电器逐个测试
- PA003/PA004：面板参数控制 K1-K4/K5-K8

### 10. IAP 功能 (`iap_function`)

| 文件 | 说明 |
| ---- | ---- |
| `iap_function.h/c` | Bootloader 跳转支持，应用向量表重映射 |

- 应用程序起始地址: `0x08005000`

### 11. 延时函数 (`delay_function`)

| 文件 | 说明 |
| ---- | ---- |
| `delay_function.h/c` | 微秒/毫秒级延时 |

---

## 外设分配

| 外设   | 用途               | 备注                          |
| ------ | ------------------ | ----------------------------- |
| USART1 | Modbus RTU 从机    | RS485, DMA 收发, PA 参数可配置波特率/校验 |
| USART3 | 编码器通讯         | RS485 半双工, DMA TX/RX       |
| TIM1   | 编码器通信调度定时  | 可配置周期 (默认 62.5μs)      |
| SPI2   | 数码管驱动         | → 74AHC595D 移位寄存器        |
| DMA1   | USART3 TX/RX       | 编码器收发                    |
| GPIO   | RS485 DE/RE, 按键, 继电器 | PB3(RS485), PB12-15(KEY), etc. |

---

## 报警码定义

| 代码 | 宏定义 | 说明 |
| ---- | ------ | ---- |
| 0x01 | `WORK_ALARM_TIMEOUT` | 通讯超时 |
| 0x02 | `WORK_ALARM_CRC_ERROR` | CRC 校验错误 |
| 0x03 | `WORK_ALARM_ADDR_ERROR` | 地址错误 |
| 0x04 | `WORK_ALARM_MODBUS_CRC` | Modbus CRC 错误 |
| 0x05 | `WORK_ALARM_NO_ENC_TYPE` | 未设置编码器类型 |
| 0x06 | `WORK_ALARM_SN_ERROR` | SN 错误 |
| 10   | — | Flash 为空 (Err.10, 首次使用) |
| 11   | — | Flash CRC 校验失败 (Err.11) |

---

## 编译与烧录

1. 使用 **Keil MDK-ARM** 打开 `MDK-ARM/` 目录下的工程文件
2. 编译 (Ctrl+F7)
3. 烧录到 STM32G491CCU3 (支持 Bootloader 跳转，应用地址 `0x08005000`)

---

## 版本信息

| 项目 | 版本 |
| ---- | ---- |
| 固件版本 | 1.0 |
| 硬件版本 | 1.0 |

---

## 更新日志

### 2026-03-03
- 修复单圈编码器通讯波特率切换导致 `TimeoutCnt` 异常跳变的编译器优化问题（补充 `volatile` 关键字）。
- 修复 RS485 半双工极速通讯下，偶发丢包导致 DMA 卡死在 `ENC_DRV_RX_WAIT` 状态的 Bug，新增超时强制清除与恢复机制。
- 将旧版固件中的多圈磁编 `Hall` 扇区（CW/CCW方向）校验完美移植并适配当前 DMA 架构，在 `0x25` 测试指令下自动触发。
- 重构转速计算逻辑，使用全局后台定时器配合各编码器解析出的单圈位置，实现 `SpeedCalc_Update` 自动计算当前滤波后转速 `ActualSpeed`。
- 将驱动层 `EncDrv_TimeoutCntUpdate` 的调用重新优化放置在 TC 接收切换间隙，恢复应用层连续掉线包容机制，极大增强了工业通讯抗扰度。

### 2026-02-26
- 新增 PA 参数 Modbus 通讯配置 (PA000 从机地址, PA001 波特率, PA002 校验位)
- 新增 PA003/PA004 面板继电器控制 (二进制掩码)
- 新增 PA005 恢复出厂设置 (输入 1234 触发)
- 新增 PA010/PA011/PA012 编码器寄存器读写 (HEX)
- 新增 DP000 固件版本显示
- 新增 DP001 继电器状态显示 (K1-K4/K5-K8 分页, b 带点区分)
- 新增 DP010 编码器寄存器读取返回值显示
- PA002 数码管显示优化: nonE / odd / EuEn 文字替代数字
- Flash 加载错误区分: Err.10(为空) / Err.11(CRC 失败)
- DTC_SaveParams_Callback 改为按参数分发操作 (不再全局 ApplyConfig)
- 优化 Flash_Storage / Parameter_Module / DigitalTube_Control 函数注释格式
- 修复开机数码管闪 0 问题 (RawData 初始化为 SEG_OFF)

### 2026-02-25
- 所有编码器 TX 函数统一使用 switch-case 风格
- 所有 TX 函数增加超时检测 (阈值 > 5)
- Key2+Key3 组合键复位时同步清除 Work_Alarm
- 所有自定义函数添加统一格式注释头

### 2026-02-09
- 初始版本，从旧工程 (`EncoderTest_V2.2`) 模块化移植
- 实现 6 种编码器协议独立模块
- 实现 Modbus RTU 从机 + 编码器寄存器路由
- 实现 DMA 非阻塞 RS485 底层驱动
- 实现数码管 + 按键状态机交互
- 实现 Flash Append-Only 参数存储
- 实现位置差分转速计算
