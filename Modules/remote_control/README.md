# App — 遥控器数据管理 (SBUS/DBUS 解析)

## 功能
- 18 字节 SBUS 帧解析 (DJI DT7/DR16)
- 5 个摇杆通道 + 2 个三档开关 + 鼠标/键盘
- ISR 中直接更新，主循环只读

## 数据结构 `RC_ctrl_t`
| 字段 | 类型 | 说明 |
|------|------|------|
| `online` | uint8_t | 1=在线 |
| `rc.ch[0]` | int16_t | 右摇杆左右 (±660) |
| `rc.ch[1]` | int16_t | 右摇杆上下 (±660) |
| `rc.ch[2]` | int16_t | 左摇杆上下 (±660) |
| `rc.ch[3]` | int16_t | 左摇杆左右 (±660) |
| `rc.ch[4]` | int16_t | 左拨轮 (±660) |
| `rc.s[0]` | RC_Switch_t | 左上开关 (1=UP,3=MID,2=DOWN) |
| `rc.s[1]` | RC_Switch_t | 右上开关 |
| `mouse` | — | 鼠标 X/Y/Z/按键 |
| `key.v` | uint16_t | 键盘按键掩码 |

## API
```c
void remote_control_init(void);           // 初始化 (调用 RC_init)
const RC_ctrl_t *get_remote_control_point(void);  // 获取数据指针 (只读)
uint8_t RC_data_is_error(void);           // 1=离线/异常
```

## 初始化
```c
#include "remote_control.h"
remote_control_init();  // 在 MX_UART5_Init() 之后调用
```

## 主循环用法
```c
const RC_ctrl_t *rc = get_remote_control_point();
if (!RC_data_is_error()) {
    // rc->rc.ch[0] ~ ch[4], rc->rc.s[0] ~ s[1] 可用
}
```
