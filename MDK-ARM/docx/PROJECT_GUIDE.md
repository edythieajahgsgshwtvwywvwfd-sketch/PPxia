# water 项目导览

## 1. 项目用途与技术栈

这是一个基于 **STM32F407VET6（Cortex-M4，168 MHz）** 的自动化设备控制工程。从当前命名和流程文字看，设备用于皮皮虾的输送、穿刺、切尾、切头、切侧身和剥壳。

工程同时保留两套构建入口：

- STM32CubeMX + CMake + GNU Arm Embedded Toolchain，供 CLion/VS Code 使用；
- Keil MDK-ARM 工程，当前源码清单比 CMake 工程更完整。

底层使用 STM32 HAL；应用采用无操作系统的“主循环 + 中断 + 多级状态机”结构。

## 2. 目录结构

```text
.
├─ Core/                  CubeMX 生成的入口、外设初始化和中断代码
├─ Drivers/               ST HAL 与 CMSIS 官方驱动
├─ MDK-ARM/
│  ├─ hardware/           设备驱动和业务状态机
│  ├─ NVIC/               定时、串口收帧与协议分发
│  ├─ system/             全局状态、延时、底层类型和 printf 重定向
│  ├─ water/              Keil 编译产物
│  └─ water.uvprojx       Keil 工程入口
├─ cmake/                 GNU Arm 工具链和 CubeMX 的 CMake 源文件清单
├─ build/                 CMake/Ninja 编译产物（当前缓存来自旧绝对路径）
├─ .vscode/               VS Code 构建、下载、调试任务
├─ water.ioc              CubeMX 硬件配置源文件
├─ CMakeLists.txt          CMake 顶层工程
├─ startup_stm32f407xx.s  GNU 工具链启动文件
└─ STM32F407XX_FLASH.ld   GNU 链接脚本
```

仓库共有约 370 个文件，但其中大量是厂商驱动或编译产物。真正需要优先阅读的是 `Core` 的入口代码、`MDK-ARM` 下的 42 个自定义 `.c/.h` 文件，以及两个 CMake 文件。

## 3. 上电后的运行链路

```text
复位
  → startup_stm32f407xx.s：设置栈、复制数据段、清零 BSS、进入 main
  → SystemInit：建立芯片基础时钟环境
  → main
      → HAL_Init
      → SystemClock_Config：12 MHz HSE → 168 MHz SYSCLK
      → MX_GPIO/DMA/TIM/UART_Init：初始化硬件
      → APP_Init：启动业务外设
      → while (1)
          → APP_MAIN
              → 读取四路光电
              → 若 system_ctrl_flag == 1：执行任务状态机并刷新 8 路直流电机 PWM
              → 刷新 OLED
              → 处理串口完成帧
```

### APP_Init 做了什么

`APP_Init()` 依次启动 TIM6 计时、串口接收、UART5 舵机控制板、OLED、直流电机 PWM、TIM5 编码器、继电器和板载舵机，并设置初始舵机角度。USART3 激光测距初始化目前被注释。

### 周期中断

TIM6 每 10 ms 进入 `HAL_TIM_PeriodElapsedCallback()`：

- `time` 递增，是大部分状态机的统一时基；
- 生成 100 ms、1 s 分频计数；
- 每 50 ms 置位 OLED 刷新标志，即约 20 Hz；
- 更新步进电机软件超时；
- 执行电机 6 的编码器位置闭环。

### UART 接收链路

- USART1：115200，调试口；使用逐字节中断接收，`printf` 也重定向到这里；
- USART2：115200，串口屏；接收 `6C 01 命令 7F` 四字节帧；
- UART4：115200，张大头/Emm_V5 步进驱动器；循环 DMA + IDLE，解析 `FD 9F` 等回包；
- UART5：9600，幻尔/Hiwonder LSC 舵机控制板；
- USART3：115200，预留给激光测距，当前业务初始化被关闭；
- USART6：源码仍保留兼容变量，但 PC6/PC7 已改作 TIM8 PWM，当前不启用。

## 4. 业务状态机

`run_mode` 决定大工序，`ctrl_flag` 决定工序内的子步骤。状态函数不能阻塞等待，而是每次主循环推进一点；电机是否到位由 `zdt_run_flag[]`、编码器标志或光电输入判断。

| run_mode | 主要作用 |
|---:|---|
| 20 | 激光基准校准入口；完成后进入 0。当前激光初始化关闭。 |
| 0 | 启动电机 3/4 输送，等待光电 2，微调步进电机 1。屏幕步骤 1“运输”。 |
| 1 | 启动直流电机 1并切换继电器。屏幕步骤 2“穿虾”。 |
| 2 | 步进电机 1/5 定位；电机 4 与直流电机 8 配合完成一次切割。屏幕步骤 3“切尾”。 |
| 3 | 电机 5 再定位，重复电机 4/8 的切割，并驱动电机 7。屏幕步骤 4“切头”、步骤 5“切侧身”。 |
| 4 | 继电器换向，移动电机 5/6，为剥壳准备。屏幕步骤 6“刺皮皮虾”。 |
| 5 | 一侧剥壳轨迹：舵机与步进电机 7/8 分段同步。屏幕步骤 7“去壳”。 |
| 6 | 另一侧镜像剥壳轨迹。 |
| 7 | 另一组电机 4/8 切割流程；当前主流程中没有明显入口。 |
| 8 | “上壳优先”模式的预定位，然后进入 6。 |
| 9 | 电机 6 转 180 度后进入 5。 |
| 10 | 电机 6 再转约 180 度，并根据剥壳顺序选择 5 或 6。 |
| 11 | 预留测试模式，当前基本为空。 |
| 12 | 电机 6 反转测试流程，当前状态链未完整收尾。 |
| 13 | 独立的后台动作：舵机、电机 3、直流电机 2 配合往返。 |

`run_mode_thirteen_update()` 在每轮 `test_all_task()` 末尾都会调用，所以模式 13 的后台动作可与前台流程并行推进。

串口屏命令含义：

| 帧 | 处理函数 | 行为 |
|---|---|---|
| `6C 01 01 7F` | `usart2_screen_command_01_handler` | 设计上用于开始流程。 |
| `6C 01 02 7F` | `..._02_handler` | 暂停；步进急停，直流 1~6 立即停，7/8 梯形减速。 |
| `6C 01 03 7F` | `..._03_handler` | 复位状态并停止所有电机。 |
| `6C 01 04 7F` | `..._04_handler` | 恢复主循环与时间计数。 |

注意：当前 `uart_execute.c` 的命令 1 分支只输出 `START` 调试文字，真正的 `usart2_screen_command_01_handler()` 调用被注释。因此上电后 `system_ctrl_flag` 默认是 0，按当前源码仅发送命令 1 不会真正启动状态机。这是现有逻辑问题，不是编码修复造成的。

## 5. 硬件资源映射

| 资源 | 用途 |
|---|---|
| TIM1 CH1~4 / PE9、PE11、PE13、PE14 | 4 路板载舵机 PWM，50 Hz。 |
| TIM2 CH3~4 / PA2、PA3 | 直流电机 5。 |
| TIM3 CH1~4 / PA6、PA7、PB0、PB1 | 直流电机 1、2。 |
| TIM4 CH1~4 / PD12~PD15 | 直流电机 3、4。 |
| TIM5 CH1~2 / PA0、PA1 | 电机 6 编码器；不再用于舵机。 |
| TIM6 | 10 ms 系统节拍。 |
| TIM8 CH1~4 / PC6~PC9 | 直流电机 7、8。 |
| TIM9 CH1~2 / PE5、PE6 | 配置为 PWM，当前业务层未明显使用。 |
| TIM12 CH1~2 / PB14、PB15 | 直流电机 6 PWM。 |
| PB10~PB13 | 4 路低电平有效光电传感器。 |
| PA5、PA8、PA11、PA12 | 4 路继电器。 |
| PB8、PB9 | 软件 I2C OLED。 |

## 6. 文件说明

### 根目录与工具配置

| 文件 | 作用 |
|---|---|
| `.editorconfig` | 统一文本文件为 UTF-8，防止中文注释再次被 ANSI/GBK 覆盖。 |
| `.mxproject` | STM32CubeMX 的辅助工程元数据。 |
| `water.ioc` | CubeMX 的权威硬件配置：芯片、引脚、时钟、DMA、定时器和串口。 |
| `CMakeLists.txt` | 顶层 CMake：建立 `water.elf`、加入自定义源码和头文件路径。 |
| `CMakePresets.json` | Debug/Release 等 Ninja 构建预设。 |
| `cmake/gcc-arm-none-eabi.cmake` | GNU Arm 交叉编译器、Cortex-M4/FPU 参数、链接选项。 |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX 生成源码与 HAL 源码清单。当前清单有缺失文件问题，见第 7 节。 |
| `startup_stm32f407xx.s` | GNU 汇编启动文件和中断向量表。 |
| `STM32F407XX_FLASH.ld` | GNU 链接布局：Flash、RAM、栈、堆、代码段和数据段。 |
| `run_flash.cmd` | 用固定本机路径的 OpenOCD + DAPLink 下载 `build/Debug/water.elf`。换电脑需修改路径。 |

`.vscode/settings.json` 让 CMake Tools 自动配置；`tasks.json` 定义清理、配置、构建、下载任务；`launch.json` 定义只构建、构建后下载和 Cortex-Debug 调试入口。这些文件也含旧用户的 OpenOCD 绝对路径。

### Core/Inc 与 Core/Src

| 文件 | 作用 |
|---|---|
| `main.c` / `main.h` | 程序入口、168 MHz 时钟配置、外设初始化顺序、错误处理。 |
| `gpio.c` / `gpio.h` | GPIO 初始电平和输入/输出模式。 |
| `dma.c` / `dma.h` | DMA 控制器时钟及各 Stream 中断优先级。 |
| `tim.c` / `tim.h` | TIM1/2/3/4/5/6/8/9/12 句柄、PWM、编码器和中断初始化。 |
| `usart.c` / `usart.h` | UART4/5、USART1/2/3 及其 DMA、GPIO、NVIC 初始化。 |
| `stm32f4xx_it.c` / `stm32f4xx_it.h` | Cortex 异常及 DMA、UART、TIM6 中断入口；把事件转交给 HAL 或自定义 IDLE 处理。 |
| `stm32f4xx_hal_msp.c` | HAL 的全局 MSP 初始化，主要是系统级时钟和中断设置。 |
| `system_stm32f4xx.c` | ST 官方系统启动与 `SystemCoreClock` 更新代码。 |
| `syscalls.c` | newlib 系统调用桩，支持 printf/堆等裸机 C 库接口。 |
| `sysmem.c` | 裸机 `_sbrk` 堆增长实现。 |
| `stm32f4xx_hal_conf.h` | HAL 模块开关、晶振频率、断言等配置。 |

### MDK-ARM/system

| 文件 | 作用 |
|---|---|
| `system.c` / `system.h` | 全局状态中心：时基、状态机标志、电机速度/位置、传感器状态、串口屏固定帧、printf 重定向。几乎所有业务模块都依赖它。 |
| `sys.c` / `sys.h` | 旧式基础类型和 Cortex-M 位带 GPIO 宏；`Stm32_Clock_Init` 等函数多为兼容层。 |
| `delay.c` / `delay.h` | 毫秒、微秒阻塞延时封装。 |

### MDK-ARM/hardware

| 文件 | 作用 |
|---|---|
| `APP_MAIN.c` / `APP_MAIN.h` | 应用层总入口：初始化、每轮主循环、run_mode 分发、OLED 调试页。 |
| `task_execute.c` / `task_execute.h` | 整机工艺状态机，是理解业务流程最关键的文件。 |
| `normal_motor.c` / `normal_motor.h` | 8 路直流电机 PWM；电机 6 编码器位置闭环；7/8 暂停减速；梯形速度规划。 |
| `Emm_V5.c` / `Emm_V5.h` | 张大头 Emm_V5 闭环步进驱动器协议：使能、速度、位置、急停、回零及运动时间估计。 |
| `servo.c` / `servo.h` | TIM1 舵机 PWM、角度映射，以及舵机与步进电机 7/8 的同步剥壳轨迹。 |
| `LobotServoController_F4.c` / `.h` | UART5 上的 LSC 16 路舵机控制板协议、收帧、单/多舵机和动作组命令。 |
| `Encoder.c` / `Encoder.h` | TIM5 编码器启动和计数读取。 |
| `jdq.c` / `jdq.h` | 4 路继电器通道到 GPIO 的映射、开关和状态同步。 |
| `gd.c` / `gd.h` | 4 路低电平有效光电传感器读取。 |
| `layser.c` / `layser.h` | 激光测距命令、串口字节解析，以及电机 1 基准位置校准；文件名 `layser` 是历史拼写。 |
| `OLED.c` / `OLED.h` | 软件 I2C OLED 绘图、文字、数字、图像等接口。 |
| `OLED_Data.c` / `OLED_Data.h` | OLED ASCII/中文字模和图像数据，体积大但基本不含控制逻辑。 |
| `motor.c` / `motor.h` | 旧的单路 1500 中位 PWM/螺旋桨接口；当前 Keil 与 CMake 均未纳入构建。 |
| `bsp_motor.c` / `bsp_motor.h` | 旧的脉冲电机方案，内容几乎全部注释；现流程改用 Emm_V5 串口驱动。 |
| `pressADC.c` / `pressADC.h` | 仅保留串口屏命令说明，当前没有 ADC 实现。 |

### MDK-ARM/NVIC

| 文件 | 作用 |
|---|---|
| `timer.c` / `timer.h` | TIM6 10 ms 调度、步进超时、绝对/相对坐标运动封装、步进回包停止判定。 |
| `UART_NVIC.c` / `UART_NVIC.h` | 六组串口接收缓存、逐字节/帧尾解析、DMA 环形缓冲、IDLE/Rx/Error 回调、主循环串口入口。 |
| `uart_execute.c` / `uart_execute.h` | 串口屏命令执行、急停/恢复、串口调试输出、屏幕文字/进度/视频更新。 |

### Drivers

`Drivers/CMSIS/Include` 是 ARM Cortex-M 内核寄存器、编译器适配、MPU/缓存等通用头文件；`Drivers/CMSIS/Device/ST/STM32F4xx/Include` 是 STM32F407 寄存器和设备定义。`LICENSE.txt` 是对应许可证。

`Drivers/STM32F4xx_HAL_Driver/Inc` 和 `Src` 是 ST HAL：

- `stm32f4xx_hal.*`：HAL 核心；
- `hal_cortex.*`：NVIC/SysTick；
- `hal_dma*`：DMA；
- `hal_exti.*`：外部中断；
- `hal_flash*`：Flash；
- `hal_gpio.*`：GPIO；
- `hal_pwr*`：电源控制；
- `hal_rcc*`：时钟树；
- `hal_tim*`：定时器/PWM/编码器；
- `hal_uart.*`：串口；
- `stm32f4xx_ll_*.h`：更接近寄存器的 LL 接口；
- `Legacy/stm32_hal_legacy.h`：旧 API 兼容宏。

这些厂商文件通常不应直接修改，应通过 CubeMX 配置或在用户代码层扩展。

### Keil 工程与产物

| 文件/目录 | 作用 |
|---|---|
| `MDK-ARM/water.uvprojx` | Keil 工程源码、宏、头文件路径和器件配置。 |
| `water.uvoptx` | Keil 目标与调试选项。 |
| `water.uvguix.*` | 各 Windows 用户的 Keil 界面状态，不影响固件逻辑。 |
| `MDK-ARM/startup_stm32f407xx.s` | Keil/ArmClang 版本启动文件。 |
| `startup_stm32f407xx.lst` | 启动汇编列表文件。 |
| `RTE/` | Keil Runtime Environment 自动生成的组件配置。 |
| `DebugConfig/`、`RTE/Device/...dbgconf*` | Keil 调试器和器件调试配置。 |
| `EventRecorderStub.scvd` | Keil Event Recorder 描述桩。 |
| `MDK-ARM/water/*.o, *.d, *.crf` | 对象、依赖和交叉引用中间文件。 |
| `water.axf`, `water.hex` | 可调试镜像和可烧录 HEX。 |
| `water.map`, `water.htm`, `water.build_log.htm` | 链接映射、调用关系和构建日志。 |
| `water.sct`, `water.lnp`, `water.dep` | Keil scatter、链接输入和依赖文件。 |

`build/Debug` 则是 CMake/Ninja 对应的 `.elf`、`.map`、`compile_commands.json`、对象文件和缓存；都属于可再生编译产物，不是源码。

## 7. 当前已确认的工程问题

1. **CMake 清单不能从零配置。** 它引用了当前目录中不存在的 `Core/Src/adc.c`、`stm32f4xx_hal_adc.c`、`stm32f4xx_hal_adc_ex.c`、`stm32f4xx_ll_adc.c` 和 `MDK-ARM/NVIC/TOF.c`。
2. **CMake 漏掉当前业务源码。** `jdq.c`、`gd.c`、`servo.c`、`Encoder.c`、`LobotServoController_F4.c` 已加入 Keil 工程，却未加入顶层 CMake。
3. **现有 `build/Debug/CMakeCache.txt` 来自另一台机器/旧目录。** 缓存记录的是 `C:/Users/32455/Desktop/stm32hal/jishe_new_4`，不能在当前路径直接复用。
4. **串口屏 START 命令未真正调用启动处理函数。** 当前只打印调试信息。
5. **暂停/恢复存在软件位置风险。** 步进命令下发时 `now_pos` 会立即改成目标值；若中途急停，软件坐标可能和真实位置不一致，源码注释已明确这一点。
6. **历史模块较多。** `motor.*`、`bsp_motor.*`、`pressADC.*` 与当前方案重复或为空，阅读时不要把它们当作现行控制链。

## 8. 推荐阅读顺序

1. `Core/Src/main.c`：先看启动顺序；
2. `MDK-ARM/hardware/APP_MAIN.c`：看主循环每轮做什么；
3. `MDK-ARM/hardware/task_execute.c`：看整机工艺流程；
4. `MDK-ARM/system/system.h`：查全局状态含义；
5. `MDK-ARM/NVIC/timer.c` 与 `UART_NVIC.c`：理解时间和通信如何推动状态机；
6. `normal_motor.c`、`Emm_V5.c`、`servo.c`：分别理解直流电机、步进电机和舵机；
7. 最后再看 `tim.c`、`usart.c`、`gpio.c` 对照实际引脚。

