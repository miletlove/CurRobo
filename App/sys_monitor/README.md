# 系统联调观察模块 (System Integration Monitor)

## 用途

在完整生产逻辑（pipeline + app_task + services）运行的同时，通过串口观察：

- **电机状态**（1Hz）：生命周期状态（OFFLINE/ENABLING/ONLINE/FAULT）、
  `online`/`enabled` 标志、反馈（位置/速度/温度/故障码）、所属 CAN 总线
- **遥控器数据**（10Hz）：5 通道、2 开关、键盘值

时基 TIM6，打印走 USART1 DMA（115200-8N1）。模块只读全局状态，
不修改控制逻辑，不发送任何 CAN 帧。

## 预期输出

```
[MON] M1 id=1 CAN1 state=OFFLINE  online=0 en=1 pos=  +0.000 vel=  +0.000 temp=0.0 fault=0x00
[MON] M2 id=2 CAN1 state=OFFLINE  online=0 en=1 pos=  +0.000 vel=  +0.000 temp=0.0 fault=0x00
[MON] M3 id=3 CAN2 state=ONLINE   online=1 en=1 pos=  +0.123 vel=  +0.010 temp=35.2 fault=0x00
[MON] M4 id=4 CAN2 state=OFFLINE  online=0 en=1 pos=  +0.000 vel=  +0.000 temp=0.0 fault=0x00
[RC] ch:    0     0     0     0     0 | sw:3/3 | key:0x0000
```

## 单电机联调（FDCAN2）说明

当前 pipeline 绑定：FDCAN1 = ID 1/2，FDCAN2 = **ID 3/4**。
测试单个挂在 FDCAN2 上的电机时：

- 电机 CAN ID 必须为 **3 或 4**（对应打印行 M3 / M4），否则请在
  `Robo/pipeline.c` 中修改绑定 ID
- FDCAN1 上没有节点，MIT 帧无人 ACK，会周期性触发
  `[CAN] FDCAN1 Bus-Off!` / `recovered` 日志——**这是 can_service
  正常工作的证据，不影响 FDCAN2 电机**；若要消除，可在 FDCAN1 上
  也挂一个节点（CAN 分析仪/另一电机）
- 联调完成后将 `sys_monitor.h` 中 `SYS_MONITOR` 改为 0 即可关闭
