# App — 遥控器数据管理 (SBUS/DBUS)

## 依赖
- `BSP/bsp_rc` — UART5 DMA 双缓冲接收

## 数据 (ISR 自动更新, 主循环只读)
| 字段 | 范围 | 含义 |
|------|------|------|
| `rc.ch[0]` | ±660 | 右摇杆左右 |
| `rc.ch[1]` | ±660 | 右摇杆上下 |
| `rc.ch[2]` | ±660 | 左摇杆上下 |
| `rc.ch[3]` | ±660 | 左摇杆左右 |
| `rc.ch[4]` | ±660 | 左拨轮 |
| `rc.s[0/1]` | 1/3/2 | 开关 UP/MID/DOWN |

## 初始化
```c
#include "remote_control.h"
remote_control_init();  // MX_UART5_Init() 之后
```

## 读取
```c
const RC_ctrl_t *rc = get_remote_control_point();
if (!RC_data_is_error()) {
    if (SW_IS_UP(rc->rc.s[0])) { /* ... */ }
}
```
