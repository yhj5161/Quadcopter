# Framework — STM32F407 四轴飞行器飞控固件

基于 STM32F407 + FreeRTOS 的四轴飞控工程，CMake + GCC ARM + OpenOCD 统一构建烧录。

当前处于**框架搭建阶段**：硬件资源注册、外设驱动、通信链路已打通并实测通过；姿态融合与 PID 控制律尚未接入。

> 本工程为 F407 单主控专用，API 层直接对接 F407 Core 实现。架构核心理念：**"同一套业务代码，芯片变了只换 Core 层"**。

## 🚀 项目定位

四轴飞控一体化固件——从传感器采集到无线遥控接收，全部集成在一个 FreeRTOS 工程里。

- 分层清晰，应用层不直接碰寄存器
- 板级接线改动只集中在 `Enroll/407_hw_config.h` 一张表里
- CMake 一键编译烧录，VS Code F7/F8
- 软 I2C/SPI 协议层与底层 HAL 分离，便于后续替换

## 🧵 任务规划

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| `ControlTask` | +3 | 1 ms | 飞控主循环 + NRF24L01 接收 |
| `SensorTask` | +2 | 2 ms (500 Hz) | ICM42688 读取；磁力计分频 20 Hz |
| `DisplayTask` | +1 | 50 ms | 遥测串口输出 |

磁力计分频 25 × 2 ms = 50 ms：软件 I2C 是阻塞式的，塞进 2 ms 节拍会挤占传感器任务。

## 📁 项目结构

```text
Quadcopter/
├─ A_Entry/                    # 程序入口（main.c + FreeRTOS 任务）
├─ API/                        # MCU 片内外设抽象接口层
│  ├─ inc/                     # gpio/adc/pwm/tim/usart 接口
│  ├─ src/
│  ├─ API_I2C/                 # 软 I2C 协议层
│  └─ API_SPI/                 # 软 SPI 协议层（模式 0）
├─ app/                        # 应用层
│  ├─ Control_Task/            # 控制任务与中断回调
│  ├─ My_Usart/                # 串口管理（收发缓冲）
│  ├─ Filter/                  # 滤波器
│  └─ PID/                     # PID 控制器
├─ BSP/                        # 板级支持层
│  ├─ ICM42688/                # 六轴 IMU（硬件 SPI2）
│  ├─ QMC5883P/                # 三轴磁力计（软 I2C）
│  ├─ NRF24L01/                # 2.4G 无线模块（软 SPI）
│  ├─ LED/                     # 状态指示灯 + 蜂鸣器
│  ├─ BMP280/                  # 气压传感器（未接入）
│  └─ KEY/                     # 按键（已从注册表移除，未使用）
├─ Core/STM32F407/             # F407 底层实现
│  ├─ src/                     # gpio/usart/tim/pwm/adc/exti/hw_spi
│  ├─ f407_soft_i2c/           # 软 I2C HAL
│  └─ f407_soft_spi/           # 软 SPI HAL
├─ Drivers/Drivers_STM32F4/    # 启动文件、CMSIS
├─ Enroll/                     # 硬件资源注册（407_hw_config.h）
├─ FreeRTOS/                   # FreeRTOS V11.1.0 内核
├─ RTOS/                       # FreeRTOSConfig.h
├─ SYSTEM/                     # 系统初始化、延时、中断优先级、总线配置
├─ OpenOCD/                    # 烧录配置
└─ docs/                       # 文档
```

## 🏗️ 分层说明

| 层级 | 目录 | 职责 |
|---|---|---|
| 入口层 | `A_Entry/` | main.c + FreeRTOS 任务创建 |
| 应用层 | `app/` | 控制任务、串口管理、滤波、PID |
| 接口层 | `API/` | 统一外设接口 + I2C/SPI 协议层 |
| 板级层 | `BSP/` | 板载器件驱动封装 |
| 注册层 | `Enroll/` | 引脚映射与资源注册 |
| 核心层 | `Core/STM32F407/` | F407 底层 GPIO/TIM/USART/PWM/ADC/SPI 实现 |
| 系统层 | `SYSTEM/` | 时钟、延时、中断优先级、总线速率 |
| 驱动层 | `Drivers/` | 启动文件、CMSIS |

### 逻辑 id 与硬件 id 分离

`Enroll` 层用 X-macro 表把「逻辑槽位」绑到「硬件外设」，业务代码只认逻辑 id：

| 宏表 | 逻辑 id | 硬件外设 |
|---|---|---|
| `HW_TIM_MAP` | `API_TIM3` | TIM5（1 ms 杂务节拍） |
| `HW_SPI_MAP` | `API_SPI2` | 无（软 SPI，PA5/PA6/PA7 + PC4 片选） |
| `HW_USART_MAP` | `API_USART1~4` | USART1 / USART2 / USART3 / UART4 |

注意 `API_TIM3`、`API_SPI2` 里的数字**只是槽位编号，与硬件编号无关**——真正的对应关系只写在 `407_hw_config.h` 的映射表里。例如硬件 SPI2（PB13/14/15）已经被 ICM42688 占用，而 `API_SPI2` 走的是软 SPI。

## 🔌 引脚分配

| 功能 | 引脚 | 状态 |
|---|---|---|
| LED1 / LED2 / LED3 | PE2 / PE3 / PE4 | ✅ 在用 |
| 蜂鸣器 | PB0 | 已注册（GPIO 输出） |
| USART1（遥测 + 串口控制） | PA9 / PA10 | ✅ 在用，115200 |
| USART2 | PD5 / PD6 | 已注册，未初始化 |
| USART3（无线串口） | PD8 / PD9 | 已注册，未初始化 |
| UART4 | PA0 / PA1 | 已注册，未初始化 |
| PWM 四路（TIM1 CH1~4） | PE9 / PE11 / PE13 / PE14 | 已注册，52.5 kHz，未驱动 |
| ADC 电池电压（ADC1_IN9） | PB1 | 已注册，未读取 |
| 软 I2C1（QMC5883P / BMP280） | PB8 (SCL) / PB9 (SDA) | ✅ 在用 |
| 软 SPI（NRF24L01） | PA5 / PA6 / PA7 / PC4(CS) + PC5(CE) | ✅ 在用 |
| 硬件 SPI2（ICM42688P, AF5） | PB12(CS) / PB13 / PB14 / PB15 | ✅ 在用 |
| 杂务定时器 | TIM5（逻辑 id `API_TIM3`） | ✅ 在用，1 ms |

## 📊 传感器与通信现状

**ICM42688P（六轴 IMU）** — 硬件 SPI2，500 Hz 轮询，姿态角经串口输出。

**QMC5883P（磁力计）** — 软 I2C1，连续模式 200 Hz / ±8 G，已完成硬铁/软铁校准，航向角可覆盖 0~360°。当前是**相对角度，零点未标定，不可当绝对航向使用**。

**NRF24L01（2.4G 遥控接收）** — 软 SPI，与遥控器端配置逐字节一致，实测已双向连通：遥控器 `TX:OK`，飞控侧稳定收包约 40 包/秒，摇杆数据实时跟随。每帧 4 字节：`d0` 右摇杆油门、`d1` 右摇杆转向、`d2` 按键码、`d3` 左摇杆倍率。

## ⚙️ 构建与烧录

- `F7`：编译，`F8`：烧录
- 命令行：`cmake --preset Debug && cmake --build --preset Debug`

## 🎯 设计原则

- 业务逻辑尽量不直接操作寄存器
- 改板级接线优先改 `Enroll/407_hw_config.h`
- BSP 接口稳定，芯片变更只换 Core 层
- 性能优先——软 I2C/SPI 拆两层，协议复用、GPIO 零开销

## ⚠️ 已知问题与待完善

### 硬件 I2C / SPI 通路尚未打通

这是当前最需要后续完善的一处架构缺口：

- `API_I2C/API_I2C.c` 与 `API_SPI/API_SPI.c` **硬编码调用 `soft_i2c_hal` / `soft_spi_hal`**，直接 `#include "soft_xxx_hal.h"`，没有任何软/硬切换的注入点。
- `Core/STM32F407/` 下只有 `f407_soft_i2c/` 和 `f407_soft_spi/` 两个软模拟 HAL，**完全没有硬件 I2C 的 Core 实现**；硬件 SPI 虽有 `f407_hw_spi.c`，但它**绕过了 `API_SPI` 协议层**，是以函数指针形式直接注入给 ICM42688 的独立通路。
- 后果：软 I2C 阻塞式收发占用 CPU，软 SPI 速度上不去；后续若要接更多高速器件（如更高速率的 IMU、外置 Flash、气压计高频采样），无法只改 `407_hw_config.h` 就切到硬件外设。
- 目标形态：给协议层补一个 HAL 函数指针表，`Enroll` 层按映射表注入软 HAL 或硬 HAL，做到"改表即换通路"——与现有 `ICM42688` 的 SPI 注入方式保持一致。

### 其他

- **飞控主循环未实现**：`ControlTask` 里姿态解算 → PID → PWM 输出仍是 `TODO`，PWM 四路只完成了初始化，从未真正驱动过。
- **NRF 接收缓冲区符号问题**：`s_nrfRxBuf` 声明为 `uint8_t`，但遥控器发送的是 `int8_t` 有符号量（-100 ~ +100）。当前打印用 `%u`，负值会显示成 226/255 一类数字；接入控制律前必须改成 `int8_t` + `%d`，否则满舵反向会读成 +255。
- **ICM 加速度 Z 轴**：静止时 roll 显示约 171°，Z 轴方向疑似取反（IMU 可能贴在 PCB 背面），需在姿态融合层处理。
- **磁力计航向未标定**：需补轴对齐、零点标定、倾斜补偿与 Yaw 融合；BMP280 气压计（定高）驱动已存在但未接入。
- **死代码**：`App_NRF24L01_TestOnce()` 从未被调用，且会改写 `SETUP_RETR` 重传配置。
- **过时注释**：`BSP/NRF24L01/NRF24L01.c` 中"（默认 1MHz）"与 `BusRate.h` 实际设置的 `API_SPI_SPEED_5M` 不符。
- **`docs/arch-guide.md` 已过时**：仍是六足蜘蛛机器人（OmniLayer）时期的架构解析，与本工程现状不符，待重写。

## 📦 分支策略

| 分支 | 定位 |
|---|---|
| `main` | 开发主线（FreeRTOS + 四轴飞控） |

## 📮 联系

QQ 邮箱：2115951478@qq.com
