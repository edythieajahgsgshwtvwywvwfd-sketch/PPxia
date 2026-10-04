# CubeMX 与 Keil 工程工作流资源

## Knowledge

- [STM32CubeMX 用户手册 UM1718](https://www.st.com/resource/en/user_manual/dm00104712-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)
  ST 官方完整手册。用于确认代码生成选项、USER CODE 保护范围和工程输出结构。
- [STM32CubeMX 代码生成概览](https://dev.st.com/stm32cube-docs/stm32cubemx/6.18.0/en/docs/markup/CubeMX_UserManual/chapters/06_6_stm32cubemx_c_code_generation_overview.html)
  ST 官方在线文档。用于理解 `Core`、`Drivers`、HAL/MSP 和外设初始化文件的来源。
- [STM32Cube 原生工程与 HAL 工作流](https://dev.st.com/stm32cube-docs/stm32cube-getting-started/1.1.2/en/markup/Ecosystem_Dev_YourApp/Ecosystem_Dev_WithSTTools.html)
  ST 官方入门流程，说明 Keil 等 IDE 与 CubeMX 重新生成代码的配合方式。
- [Keil Assistant 原始项目说明](https://github.com/github0null/keil-assistant)
  插件作者仓库，说明它通过调用 Keil 命令行完成编译和下载，以及推荐打开 `.uvprojx` 所在目录。原项目已归档。
- [Arm Keil Studio Pack](https://marketplace.visualstudio.com/items?itemName=Arm.keil-studio-pack)
  Arm 官方 VS Code 工具链。仅在未来决定从旧 Keil Assistant 迁移时参考，当前不要求迁移。

## Wisdom (Communities)

- [STMicroelectronics Community](https://community.st.com/)
  适合核实特定 CubeMX 版本的生成行为、HAL 外设问题和已知缺陷。
- [Arm Keil forum](https://community.arm.com/support-forums/f/keil-forum)
  适合咨询 µVision、ArmClang、设备包和下载器相关问题。
