# Mission: 安全维护 CubeMX 与 Keil Assistant STM32 工程

## Why
在 VS Code 中高效编写和下载 STM32F407 固件，同时能用 CubeMX 修改引脚、时钟和外设，而不覆盖业务代码或破坏 Keil 工程。

## Success looks like
- 能区分完整 CubeMX 工程根目录与 `MDK-ARM` Keil 子工程
- 能判断一个文件应由 CubeMX 修改还是由自己直接编辑
- 每次重新生成后都能检查、编译并安全下载
- 能在 VS Code 中看到完整工程，同时继续用 Keil Assistant 构建

## Constraints
- 日常编辑器为 VS Code
- 构建和下载继续使用 Keil/Keil Assistant
- 当前 CMake 工程清单不完整，暂不作为权威构建入口

## Out of scope
- 暂不迁移到 RTOS
- 暂不把 Keil 工程整体迁移为 CMake 或 CMSIS Solution
