import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

# ================= 配置区域 =================
# 与 C 代码宏定义保持一致 (PA_SIZE = 60)
FLASH_PAGE_SIZE = 2048
PA_SIZE = 60
HEADER_SIZE = 8
FOOTER_SIZE = 8

# 计算衍生值
DATA_SIZE = PA_SIZE * 4  # 60 * 4 = 240 Bytes
RECORD_SIZE = HEADER_SIZE + DATA_SIZE + FOOTER_SIZE  # 8 + 240 + 8 = 256 Bytes
RECORDS_PER_PAGE = FLASH_PAGE_SIZE // RECORD_SIZE  # 2048 // 256 = 8

# 设置中文字体 (根据系统环境调整)
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial'] 
plt.rcParams['axes.unicode_minus'] = False

def draw_flash_layout():
    # 创建画布
    fig, ax = plt.subplots(figsize=(10, 10)) # 高度稍微加大一点到 10
    
    # 绘制背景：整个 Flash 页 (2KB)
    page_height = RECORDS_PER_PAGE * 70 + 80
    ax.add_patch(patches.Rectangle((0, 0), 650, page_height, linewidth=2, edgecolor='#333333', facecolor='#f9f9f9', linestyle='--'))
    ax.text(325, page_height + 10, 'Flash Page (2KB / 2048 Bytes)', ha='center', va='bottom', fontsize=12, fontweight='bold')
    
    colors = ['#FFD700', '#90EE90', '#87CEFA', '#FFB6C1', '#DDA0DD', '#F08080', '#E0FFFF', '#FAFAD2']
    
    # 从上往下绘制每一条记录
    for i in range(RECORDS_PER_PAGE):
        y_pos = page_height - 80 - (i * 70)
        start_x = 40
        addr_offset = i * RECORD_SIZE
        
        # 1. 地址
        ax.text(start_x - 10, y_pos + 25, f'+0x{addr_offset:03X}\n({addr_offset})', ha='right', va='center', fontsize=9, color='blue', family='monospace')

        # 2. Header
        ax.add_patch(patches.Rectangle((start_x, y_pos), 60, 50, facecolor='#FF6347', edgecolor='black'))
        ax.text(start_x + 30, y_pos + 25, 'Header\n(8B)', ha='center', va='center', fontsize=8, color='white', fontweight='bold')
        
        # 3. Data
        ax.add_patch(patches.Rectangle((start_x + 60, y_pos), 350, 50, facecolor=colors[i % len(colors)], edgecolor='black'))
        label_text = f'记录 #{i+1} (Index={i+1})\n数据区 ({DATA_SIZE} Byte)\nPA[0]...PA[{PA_SIZE-1}]'
        ax.text(start_x + 235, y_pos + 25, label_text, ha='center', va='center', fontsize=9)
        
        # 4. Footer
        ax.add_patch(patches.Rectangle((start_x + 410, y_pos), 60, 50, facecolor='#4682B4', edgecolor='black'))
        ax.text(start_x + 440, y_pos + 25, 'Footer\n(8B)', ha='center', va='center', fontsize=8, color='white', fontweight='bold')

        # 5. 右侧备注
        ax.text(start_x + 480, y_pos + 25, f'= {RECORD_SIZE} 字节', ha='left', va='center', fontsize=9, color='#555555')

    # 计算剩余空间
    used_space = RECORDS_PER_PAGE * RECORD_SIZE
    remain_space = FLASH_PAGE_SIZE - used_space
    
    # 绘制底部信息栏
    info_text = (
        f"配置参数:\n"
        f"  - Flash 页大小: {FLASH_PAGE_SIZE} 字节\n"
        f"  - PA参数个数: {PA_SIZE} 个 (int32)\n"
        f"  - 单条记录大小: {RECORD_SIZE} 字节 (8+240+8)\n"
        f"状态统计:\n"
        f"  - 可存记录数: {RECORDS_PER_PAGE} 条 / 页\n"
        f"  - 空间利用率: 100% (完美填充)\n"
        f"  - 剩余浪费: {remain_space} 字节"
    )
    
    # 【修改点 1】：位置 y 设为 -50，va='top' 表示文本框顶部对齐 -50
    # 这样文本框就会完全跑到图形下方，不会遮挡任何东西
    ax.text(40, -50, info_text, ha='left', va='top', fontsize=10, 
            bbox=dict(boxstyle="round,pad=0.5", facecolor='#e6f3ff', edgecolor='#0066cc', alpha=0.9))

    # 设置坐标轴范围
    ax.set_xlim(-100, 650)
    # 【修改点 2】：Y轴下限设为 -250，给下方的文字留出足够的显示空间
    ax.set_ylim(-250, page_height + 30)
    
    ax.set_title(f'Flash存储布局 (PA_SIZE={PA_SIZE}, Perfect Alignment)', fontsize=14, pad=20)
    ax.axis('off')

    plt.tight_layout()
    
    # 自动保存到脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in globals() else os.getcwd()
    save_path = os.path.join(script_dir, 'flash_layout_perfect.png')
    
    plt.savefig(save_path, dpi=150)
    print(f"图表已生成: {save_path}")

if __name__ == "__main__":
    try:
        draw_flash_layout()
    except Exception as e:
        print(f"绘图出错: {e}")