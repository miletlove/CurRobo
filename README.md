
# CurRobo — 四足机器人主控系统

基于 **STM32H723VGT6** 的四足/多足机器人嵌入式主控固件，采用 CAN/FDCAN 总线驱动 CyberGear 系列电机，支持 DBUS 遥控器控制与 WS2812 RGB LED 状态指示。

[![MCU](https://img.shields.io/badge/MCU-STM32H723VGT6-blue)](https://www.st.com)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](https://www.freertos.org)
[![Toolchain](https://img.shields.io/badge/Toolchain-GCC--ARM--none--eabi-orange)](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
[![Build](https://img.shields.io/badge/Build-CMake%2BNinja-8A2BE2)](https://cmake.org)
[![Hardware](https://img.shields.io/badge/Hardware-DM--MC--Board02-red)]()

---

## 📁 目录结构

```
CurRobo/
├── Core/                    # STM32CubeMX 自动生成 HAL 代码
│   ├── Inc/                 # 外设头文件 (main.h, gpio.h, fdcan.h...)
│   └── Src/                 # 外设源文件 (main.c, gpio.c, fdcan.c...)
├── BSP/                     # 板级支持包 (Board Support Package)
│   ├── Inc/                 # bsp_rc.h, bsp_usart.h, can_bsp.h
│   └── Src/                 # 遥控器DMA接收, USART, FDCAN初始化和收发
├── App/                     # 应用层模块
│   ├── Inc/                 # cybergear_motor.h, remote_control.h, ws2812.h
│   └── Src/                 # 电机控制, 遥控器解析, LED驱动
├── cmake/                   # CMake 交叉编译工具链
│   ├── gcc-arm-none-eabi.cmake
│   └── starm-clang.cmake
├── build/                   # 构建输出目录
├── Docs/                    # 文档与参考例程
│   ├── motor/               # 电机协议文档
│   └── 例程/                # 达妙科技官方外设例程
├── CMakeLists.txt           # 顶层 CMake 构建脚本
├── CMakePresets.json        # CMake 预设配置 (Debug/Release)
├── STM32H723XG_FLASH.ld     # 链接脚本
├── startup_stm32h723xx.s    # 启动汇编文件
├── CurRobo.ioc              # STM32CubeMX 工程文件
└── README.md
```

## ⚙️ 主要功能

| 模块 | 功能说明 | 核心文件 |
|------|---------|----------|
| 🎮 **遥控器解析** | DBUS (SBUS) 协议解析, DMA 双缓冲接收 | `App/Inc/remote_control.h`, `BSP/Src/bsp_rc.c` |
| 🏃 **电机控制** | CyberGear 系列 FOC 电机驱动, MIT 运控模式 | `App/Inc/cybergear_motor.h` |
| 🌐 **CAN 通信** | FDCAN Classic Mode, 1Mbps, 29bit 扩展帧 | `BSP/Inc/can_bsp.h`, `Core/Src/fdcan.c` |
| 💡 **RGB LED** | WS2812B 状态指示, SPI6 单线模拟协议 | `App/Inc/ws2812.h` |
| 🔌 **串口通信** | UART 多路串口配置与收发 | `BSP/Src/bsp_usart.c` |

## 🔧 开发环境

### 硬件平台

- **主控芯片**: STM32H723VGT6 (Cortex-M7, 550MHz)
- **开发板**: 达妙科技 DM-MC-Board02 (CtrBoard-H7)
- **电机**: CyberGear 系列 FOC 伺服电机 ×6
- **遥控器**: 大疆 DT7 遥控器 (DBUS 协议)

### 软件环境

| 工具 | 版本/说明 |
|------|----------|
| CMake | ≥ 3.22 |
| GCC ARM Toolchain | `arm-none-eabi-gcc` (推荐 GNU Arm Embedded 10.3+) |
| Ninja | 构建生成器 |
| STM32CubeMX | STM32H723VGT6 外设配置与初始化代码生成 |

## 🚀 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/miletlove/CurRobo.git
cd CurRobo
```

### 2. 配置构建

```bash
# 使用 CMake Preset 配置 Debug 模式
cmake --preset Debug

# 或手动配置
cmake -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
```

### 3. 编译

```bash
cmake --build --preset Debug
# 或
ninja -C build/Debug
```

### 4. 烧录

使用 STM32CubeProgrammer、OpenOCD 或 J-Link 将 `build/Debug/test.elf` (或 `.bin`) 烧录到目标板上:

```bash
# 示例: 使用 OpenOCD + ST-Link
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Debug/test.elf verify reset exit"
```

### 5. 调试

推荐使用 [Ozone](https://www.segger.com/products/development-tools/ozone-j-link-debugger/) (J-Link) 进行调试, 配置文件 `ozone.jdebug` 已包含在工程中。

## 📖 核心协议

### CyberGear 电机通信协议

电机通过 FDCAN 扩展帧 (29-bit ID) 进行通信:

| 通信类型 | CAN ID 格式 | 数据方向 |
|---------|------------|---------|
| MIT 控制 | `(0x01 << 24) \| (CAN_ID << 8) \| motor_id` | 主机 → 电机 |
| 电机反馈 | `(0x02 << 24) \| (CAN_ID << 8) \| motor_id` | 电机 → 主机 |
| 使能/停止 | `(0x03/0x04 << 24) \| (CAN_ID << 8) \| motor_id` | 主机 → 电机 |

> 详细协议参见 `Docs/motor/CyberData.md`

### DBUS 遥控器协议

- 波特率: 100000 bps
- 帧长度: 18 字节
- 接收方式: UART5 DMA 双缓冲 (Double Buffer Mode)
- 数据解析: 通道值量程 [0, 1684] 偏移 1024

## 🎯 运行模式

| 模式 | 说明 |
|------|------|
| **MIT 运控模式** | 上电默认模式, 支持位置/速度/力矩三环控制 |
| **位置模式** | 电机位置闭环, 可设置 KP/KD 参数 |
| **速度模式** | 电机速度闭环控制 |
| **零位校准** | 设置电机当前机械角度为零位 |

## 📝 开发规范

- **C 标准**: C11
- **命名风格**:
  - 文件名: `snake_case` (如 `cybergear_motor.h`)
  - 函数名: `snake_case` (如 `cybergear_motor_init()`)
  - 类型名: `PascalCase_t` (如 `CyberGear_Motor_t`)
  - 宏: `UPPER_CASE` (如 `CG_P_MIN`)
- **模块分层**: BSP → App, 上层不直接操作 HAL 寄存器
- **注释语言**: 中文

## 📚 参考资源

- [STM32H723/733 Reference Manual (RM0468)](https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [CyberGear Motor Protocol](Docs/motor/CyberData.md)
- [DJI DBUS Protocol](https://www.robomaster.com)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [达妙科技 CtrBoard-H7 例程](Docs/例程/)

## 📄 License

本项目基于 STM32CubeMX 生成代码开发, 遵循 STMicroelectronics 相关许可协议。

---

> **CurRobo** — *让机器人控制更简单, 更可靠.*
