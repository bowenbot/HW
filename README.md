# SuperPower 校内赛 嵌入式代码

## 一、主要任务

本仓库为同济大学 SuperPower 战队 2027 赛季校内赛嵌入式方向代码。
基于 RoboMaster C 型开发板，完成以下任务：

1. C 板基本功能：蜂鸣器控制、LED 流水灯、串口打印 IMU 三轴、DT7 遥控器通讯。
2. 姿态与电机联动：
   - 右拨杆下档：失能，所有电机无力。
   - 右拨杆中档：姿态联动，A 电机与 C 板 yaw 1:1 跟随，B 电机比例由左拨杆决定。
   - 右拨杆上档：复位，两台电机 R 标对齐 C 板 R 标。
   - 左拨杆下档：B 比例 0.5；中档：-1；上档：3。
   - 支持手动转动任一电机时，另一台按比例跟随，C 板 yaw 不变，参考零点自动更新，不回正。
3. 自动折叠决策：使用红外测距模块测量隧道距离，自动折叠云台通过净高 400mm 隧道。
4. 24V 转 5V 降压模块：基于 MP4420A 设计原理图与 PCB。

## 二、硬件平台

- 主控：STM32F407IGH6（RoboMaster C 型开发板）
- IMU：BMI088（SPI1）
- 电机：RM6020 × 2（CAN1）
- 遥控器：DT7 + DR16（DBUS / USART3）
- 降压模块：MP4420A（24V 转 5V，自制）

## 三、软件架构

- 框架：STM32 HAL 库 + FreeRTOS（CMSIS_V1）
- 构建：CMake + ARM GCC
- 语言：C++（业务逻辑）+ C（HAL 底层）
- 编辑器：VSCode + CMake Tools + Cortex-Debug

## 四、目录结构

- `applications/` — 各功能任务（imu_task、uart_task、can_task、gimbal_task、led_task、buzzer_task、plotter_task）
- `Src/`、`Inc/` — CubeMX 生成的主程序和外设初始化
- `Drivers/` — ST HAL 库
- `Middlewares/` — 板级支持
- `sp_middleware/` — 同济 SuperPower 中间件
- `cmake/` — CMake 构建配置
- `CMakeLists.txt` — 顶层构建脚本
- `openocd.cfg` — 烧录器配置

## 五、功能说明

| 功能 | 实现方式 |
|---|---|
| 蜂鸣器 | TIM4_CH3 PWM，开机响 3 声 |
| LED | TIM5 三通道 PWM，红绿蓝流水灯 |
| IMU | BMI088 + Mahony，1ms 更新 |
| 遥控器 | USART3 + DMA 空闲中断，解析 18 字节 DBUS 帧 |
| 电机 | CAN1 收发，GM6020 电流控制，双环 PID（位置环 + 速度环） |
| 云台联动 | 状态机 + 手动检测 + 参考零点动态更新 |
| 波形输出 | Plotter 中间件，UART1 输出 IMU 三轴，用 serialplot 查看 |

## 六、操作注意事项

1. 右拨杆下档为失能模式，调试前必须验证全部电机无力。
2. 电机测试必须轮子离地或机构架空。
3. 24V 上电必须限流，首次通电前检查电源极性、线束绝缘、短路风险。
4. 烧录、接线、拆装前必须切断动力电源。
5. 出现异味、冒烟、失控、线束拉扯时立即断电。
6. 电机拨码开关：A 电机 ID=1，B 电机 ID=2，第 4 位 ON（终端电阻）。
7. 上电顺序：先接好所有线，确认正负极，再开电源。

## 七、开发历程与重大变更

### 第一阶段（作废）

第一版代码按单环 PID 设计，在实机调试中出现以下问题：

- 速度环振荡剧烈，无法收敛。
- 手动转动检测误触发频繁，导致联动中断。
- 遥控器 DBUS 通信使用错误的接收回调，无法接收数据。
- CAN 帧 ID 计算错误，电机无响应。

以上问题在调试中逐渐暴露，累积过多，修补成本高于重写。经导师确认，决定废弃第一版，从零重构。

### 第二阶段（当前版本）

在新版本中做了以下结构性调整：

- PID 改为双环（位置环 + 速度环），分别独立调参。
- 遥控器改用 DMA + 空闲中断，正确的回调函数。
- CAN 帧 ID 修正，电机 ID 与拨码一致。
- 手动转动检测改为 100ms 窗口判断，避免误触发。
- 逐电机独立调参，先 A 后 B，再开联动。

调试记录和参数整定过程保留在提交历史中。

## 八、第三方代码来源、许可证与修改内容

### 1. sp_middleware

- 来源：https://github.com/TongjiSuperPower/sp_middleware
- 用途：BMI088 驱动、CAN 通信、DBUS 解析、RM 电机控制、PID、Mahony 等。
- 许可证：以原仓库声明为准。
- 修改内容：未修改源码，仅通过 CMake 选择性编译所需模块。

### 2. FreeRTOS

- 来源：https://www.freertos.org/
- 用途：任务调度，为 CMSIS-RTOS 提供支持。
- 许可证：MIT License。
- 修改内容：未修改，由 CubeMX 自动集成。

### 3. CMSIS-RTOS

- 来源：ARM 官方，随 STM32CubeF4 固件包提供。
- 用途：FreeRTOS 的 CMSIS 封装接口。
- 许可证：Apache-2.0。
- 修改内容：未修改。

### 4. STM32 HAL 库

- 来源：STMicroelectronics，随 STM32CubeF4 固件包提供。
- 用途：外设驱动。
- 许可证：BSD-3-Clause。
- 修改内容：未修改。

## 九、版本记录

- V1.0.0（已作废）：第一版实现，单环 PID，存在速度环振荡、遥控器通信失败等问题。
- V2.0.0（当前）：重构版本，双环 PID，遥控器 DMA 接收，逐电机调参。

## 十、Git 提交规范

- 前缀：`feat` / `fix` / `docs` / `chore` / `build` / `refactor`
- 每完成一个模块提交一次，提交历史反映开发过程。
- 重要功能保留可回退的稳定版本。