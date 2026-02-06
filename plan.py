很好的简化！既然只需要保存 `PA` 参数（用户设置），`DP`（显示参数）不需要保存，那么方案会变得更加高效和轻量。

这是为您定制的 **STM32G4 Flash 均衡磨损存储方案 (PA参数专用版)**。

---

### 一、 实施计划 (The Plan)

我们将利用 **Flash 的最后两页**（Page 126 和 Page 127）作为存储区，采用 **乒乓（Ping-Pong）机制** 来实现追加写入和磨损均衡。

#### 1. 物理布局 (Memory Map)

* **MCU**: STM32G491CCU3 (256KB Flash)
* **Page A (主存储区)**: `0x0803 F000` (倒数第2页)
* **Page B (备份/搬运区)**: `0x0803 F800` (倒数第1页)
* **记录格式**: 每条记录占用 **8字节 (64bit)**，正好符合 STM32G4 的写入位宽。
* `[ 参数ID (2byte) | 参数值 (4byte) | 校验码 (2byte) ]`



#### 2. 运行机制 (Workflow)

* **上电加载 (Load)**:
1. 扫描 Page A 和 Page B，判断哪个是“当前有效页”（非空的那个）。
2. 从头到尾读取该页的所有记录。
3. 每读到一条 `PA_xx`，就更新 RAM 中的 `PA_Buffer`。
4. *结果*：RAM 里保留的是 Flash 中最后一次写入的最新值。


* **修改参数 (Append Write)**:
1. 当长按 Key 4 保存时，查找当前页的**下一个空白位置 (0xFF...)**。
2. 写入一条新记录：`ID=当前参数号, Value=新值`。
3. *优势*：不擦除，速度极快（微秒级）。


* **页面写满 (Garbage Collection / Swap)**:
1. 当当前页（比如 Page A）写满了（存了 256 次修改）。
2. 擦除 Page B。
3. 把 RAM 中当前 **所有 50 个 PA 参数** 的最新值，整齐地写到 Page B 的开头。
4. 擦除 Page A。
5. 标记 Page B 为当前页。
6. *优势*：清理了历史冗余数据，Flash 焕然一新。



---

### 二、 完整代码实现

请将以下代码复制到 `DigitalTube_Control.c` 的 **末尾**。同时确保在 `main.c` 初始化时调用 `Load_PA_From_Flash()`。

```c
// ========================================================================================
//                             Flash 均衡磨损存储系统 (PA专用版)
// ========================================================================================

// ---------------- 配置区域 ----------------
// STM32G491CC (256KB) 最后一页地址 = 0x08040000 - 2KB
#define FLASH_PAGE_SIZE     2048
#define FLASH_PAGE_A_ADDR   0x0803F000  // 倒数第二页
#define FLASH_PAGE_B_ADDR   0x0803F800  // 倒数第一页

// 记录结构体 (严格对齐 8字节/64bit)
// [ ID(2) | Value(4) | Magic(2) ]
typedef struct {
    uint16_t ParamID;  // PA参数编号 (0 ~ PA_SIZE-1)
    int32_t  Value;    // 参数值
    uint16_t Magic;    // 固定为 0xA55A
} Flash_Record_t;

#define RECORD_MAGIC  0xA55A

// ---------------- 内部状态变量 ----------------
static uint32_t g_ActivePageAddr = FLASH_PAGE_A_ADDR; // 当前正在使用的页

// ---------------- 内部函数声明 ----------------
static void Flash_Erase_Page(uint32_t page_addr);
static void Flash_Write_Record(uint32_t addr, uint16_t id, int32_t val);

/****************************************************************************************
* 函数名称：Flash_Find_Active_Page
* 函数功能：上电时判断哪一页是有效页
****************************************************************************************/
static void Flash_Find_Active_Page(void)
{
    // 读取两页的第一个字
    uint32_t headerA = *(__IO uint32_t*)FLASH_PAGE_A_ADDR;
    uint32_t headerB = *(__IO uint32_t*)FLASH_PAGE_B_ADDR;

    // 逻辑：谁有数据谁就是 Active。如果都有数据(异常)或都空，默认用 A。
    if (headerA != 0xFFFFFFFF && headerB == 0xFFFFFFFF) {
        g_ActivePageAddr = FLASH_PAGE_A_ADDR;
    } 
    else if (headerB != 0xFFFFFFFF && headerA == 0xFFFFFFFF) {
        g_ActivePageAddr = FLASH_PAGE_B_ADDR;
    } 
    else {
        // 全空(第一次使用) 或 全满(异常)，重置为 A
        g_ActivePageAddr = FLASH_PAGE_A_ADDR;
    }
}

/****************************************************************************************
* 函数名称：Load_PA_From_Flash
* 函数功能：[核心] 上电读取，恢复 PA 参数
* 说明：    需在 main.c 初始化时调用
****************************************************************************************/
void Load_PA_From_Flash(void)
{
    // 1. 确定当前页
    Flash_Find_Active_Page();
    
    uint32_t curr_addr = g_ActivePageAddr;
    uint32_t end_addr = g_ActivePageAddr + FLASH_PAGE_SIZE;

    // 2. 扫描整页日志
    while (curr_addr < end_addr) {
        uint64_t raw_data = *(__IO uint64_t*)curr_addr;
        
        // 遇到空白，说明后面没数据了
        if (raw_data == 0xFFFFFFFFFFFFFFFF) break;

        Flash_Record_t *rec = (Flash_Record_t*)&raw_data;

        // 校验魔术字，防止读取到写坏的数据
        if (rec->Magic == RECORD_MAGIC) {
            // 合法性检查：ID必须在 PA_SIZE 范围内
            if (rec->ParamID < PA_SIZE) {
                // 覆盖 RAM 中的旧值 (日志靠后的才是最新的)
                PA_Buffer[rec->ParamID] = rec->Value;
            }
        }
        
        curr_addr += 8; // 下一条
    }
}

/****************************************************************************************
* 函数名称：DTC_SaveParams_Callback
* 函数功能：[核心] 保存单个参数 (追加写入模式)
* 说明：    由 Key 4 长按触发。只保存当前修改的那个参数，速度极快。
****************************************************************************************/
void DTC_SaveParams_Callback(void)
{
    // 如果修改的是 DP 组，不保存，直接返回
    if (DTC_Dev.GroupIdx != 0) return;

    uint16_t save_id = DTC_Dev.ParamNum;
    int32_t  save_val = PA_Buffer[save_id]; // 注意：此时 Buffer 已被 EditVal 更新

    uint32_t write_addr = g_ActivePageAddr;
    uint32_t end_addr = g_ActivePageAddr + FLASH_PAGE_SIZE;
    uint8_t page_is_full = 1;

    // 1. 寻找空白位置
    while (write_addr < end_addr) {
        if (*(__IO uint64_t*)write_addr == 0xFFFFFFFFFFFFFFFF) {
            page_is_full = 0;
            break;
        }
        write_addr += 8;
    }

    // 2. 如果页没满，直接追加写入
    if (!page_is_full) {
        Flash_Write_Record(write_addr, save_id, save_val);
    }
    // 3. 如果页满了，执行“搬家 (Page Swap)”
    else {
        // 确定搬家目标页
        uint32_t target_page = (g_ActivePageAddr == FLASH_PAGE_A_ADDR) ? FLASH_PAGE_B_ADDR : FLASH_PAGE_A_ADDR;
        
        // A. 擦除目标页
        Flash_Erase_Page(target_page);
        
        // B. 将 RAM 中所有的 PA 参数快照写入目标页 (整理碎片)
        uint32_t target_write_ptr = target_page;
        for (int i = 0; i < PA_SIZE; i++) {
            Flash_Write_Record(target_write_ptr, i, PA_Buffer[i]);
            target_write_ptr += 8;
        }

        // C. 擦除旧页 (标记为弃用)
        Flash_Erase_Page(g_ActivePageAddr);

        // D. 切换当前页指针
        g_ActivePageAddr = target_page;
    }
}

// ---------------- 底层 Flash 辅助函数 ----------------

static void Flash_Erase_Page(uint32_t page_addr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef Erase;
    uint32_t PageError;
    
    Erase.TypeErase = FLASH_TYPEERASE_PAGES;
    Erase.Banks = FLASH_BANK_1;
    // 计算页号: (Addr - Base) / 2048
    Erase.Page = (page_addr - 0x08000000) / FLASH_PAGE_SIZE;
    Erase.NbPages = 1;
    
    HAL_FLASHEx_Erase(&Erase, &PageError);
    HAL_FLASH_Lock();
}

static void Flash_Write_Record(uint32_t addr, uint16_t id, int32_t val)
{
    Flash_Record_t rec;
    rec.ParamID = id;
    rec.Value = val;
    rec.Magic = RECORD_MAGIC;

    HAL_FLASH_Unlock();
    // STM32G4 必须按 64位 (Double Word) 写入
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, *(__IO uint64_t*)&rec);
    HAL_FLASH_Lock();
}

```

### 三、 必要的微调 (Checklist)

1. **头文件声明**:
在 `DigitalTube_Control.h` 中添加 `Load_PA_From_Flash` 的声明，以便 main 函数调用。
```c
void Load_PA_From_Flash(void); // 添加这行

```


2. **主函数调用**:
在 `main.c` 的 `while(1)` 之前，在外设初始化之后：
```c
/* USER CODE BEGIN 2 */
DTC_Init();
Load_PA_From_Flash(); // <--- 必须加这句，上电加载参数
/* USER CODE END 2 */

```


3. **默认值逻辑**:
如果芯片是全新的，Flash 也是空的，`Load_PA_From_Flash` 什么都不会做。
所以，你需要在 `DTC_Init` 或者定义 `PA_Buffer` 数组时给一个由编译器生成的默认值。
```c
// 在 DigitalTube_Control.c 顶部
int32_t PA_Buffer[PA_SIZE] = { 100, 200, 500, ... }; // 你的出厂默认值

```



### 四、 这个方案的优势

1. **超长寿命**:
一页可以存  条记录。
每页写满才擦除一次。
寿命 = 。
哪怕用户是个“参数狂魔”，这台设备也能传家了。
2. **掉电安全**:
因为我们只保存 `PA`。在搬家时，我们是先写入新页，再擦除旧页。
如果在写入新页时断电 -> 旧页还在，数据不丢。
如果在擦除旧页时断电 -> 新页已经好了，数据不丢。
3. **极速响应**:
平时保存只需要写 8 个字节，耗时几十微秒，数码管显示完全不会闪烁或卡顿。