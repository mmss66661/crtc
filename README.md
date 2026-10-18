# CRTC_CAR_DEMO

基于 **STM32F103C8T6** 的 PS2 手柄遥控小车，参加**重庆大学机器人训练大赛 CRTC2026**。
HAL 库 + CMake/Ninja 构建，C/C++ 混合编程。

PS2 手柄经 SPI 时序读入摇杆与按键，主循环解算后驱动 4 路电机 PWM，实现前进、后退与差速转向。

---

## 演示视频

两段实拍记录：PS2 手柄遥控下底盘的起动、转向与停止。均为 **1920×1080 / 60 fps / H.264 + AAC**，满足手册 4.5 对清晰度的要求。

### 演示一：手柄遥控整机运动

<video src="docs/media/chassis-remote-01.mp4" controls width="720"></video>

> [`docs/media/chassis-remote-01.mp4`](docs/media/chassis-remote-01.mp4) —— 4.48 s / 12.2 MB

### 演示二：手柄遥控连续行驶

<video src="docs/media/chassis-remote-02.mp4" controls width="720"></video>

> [`docs/media/chassis-remote-02.mp4`](docs/media/chassis-remote-02.mp4) —— 10.45 s / 26.8 MB

> GitHub 会过滤 README 中的 `<video>` 标签，网页端请点击上方链接播放；VS Code / Typora 等本地预览可内联播放。

四轮电机开环 PWM 的单独演示见 [`docs/media/chassis-motion.mp4`](docs/media/chassis-motion.mp4)，硬件设计稿与实物见 [`docs/硬件开发记录.md`](docs/硬件开发记录.md)。

---

## 仓库结构

```
.
├── README.md                     本文件：项目概览与开发指引
├── docs/                         参赛文档（按 CRTC2026 手册 4.3 规范存放）
│   ├── README.md                 文档索引 + 中期考核指标完成情况
│   ├── 01-整体方案与硬件设计.md
│   ├── 02-底盘运动.md            含演示视频
│   ├── 03-PS2手柄遥控.md
│   ├── 04-开发日志与问题排查.md
│   ├── 05~10                     电控 / 硬件 / 机械三组「学习记录 + 进度记录」
│   └── media/                    演示视频与硬件图片
├── Chassis/                      底盘层：电机控制 + 差速解算 + 板级初始化
├── PS2/                          PS2 手柄驱动
├── Core/                         CubeMX 生成的 HAL 初始化
├── Drivers/                      STM32 HAL / CMSIS
├── cmake/                        构建配置
├── CMakeLists.txt                构建入口
└── flash.bat                     一键烧录脚本（ST-LINK）
```

> 详细设计说明与技术决策从 `docs/README.md` 的索引进入。

---

## 硬件

| 项目 | 参数 |
| --- | --- |
| MCU | STM32F103C8T6（Cortex-M3，64 KB Flash / 20 KB SRAM） |
| 系统时钟 | HSE 8 MHz × PLL9 = **72 MHz**（AHB 72 / APB1 36 / APB2 72） |
| 电机驱动 | TIM1 CH1~CH4 输出 PWM，20 kHz（72 MHz / 3600），IN2 与 STBY 由 GPIO 控制 |
| 手柄接口 | PS2 接收器，接 PA4~PA7（当前固件未启用） |
| 调试接口 | SWD（PA13 / PA14） |

| MCU 引脚 | 网络名 | 功能 |
| --- | --- | --- |
| PA8 / PB12 | FL_IN1 / FL_IN2 | 左前电机（CH1 + GPIO） |
| PA9 / PB13 | FR_IN1 / FR_IN2 | 右前电机（CH2 + GPIO） |
| PA10 / PB14 | BL_IN1 / BL_IN2 | 左后电机（CH3 + GPIO） |
| PA11 / PB15 | BR_IN1 / BR_IN2 | 右后电机（CH4 + GPIO） |
| PB0 / PB1 | STBY1 / STBY2 | 前级 / 后级驱动使能（高有效） |
| PA4~PA7 | DI / CMD / CS / CLK | PS2 接收器（软件模拟时序） |

> 板上 PB3 / PB4 / PB5 / PB6 为空脚，`MX_TIM2_Init` / `MX_TIM3_Init` / `MX_SPI1_Init` 已停用。

---

## 软件结构

```
Core/               STM32CubeMX 生成的 HAL 初始化（时钟、GPIO、中断）
Chassis/            底盘层（板级引脚初始化在此完成，不依赖 tim.c / spi.c）
  Inc/Chassis.h       Motor 类（PWM + 方向脚）+ ChassisCtrl（差速解算）
  Src/Chassis.cpp     TIM1 PWM 初始化、STBY 使能、限幅、左右轮分配
PS2/                PS2 手柄驱动（当前固件未启用，引脚待适配）
cmake/              CubeMX 生成的 CMake 目标
```

### 控制流程

```
update(v, turn) ──> ChassisCtrl::update()  ──> Motor::set_speed()
                       left  = v - turn/2       正转：IN2 = 0, CCR = |speed|
                       right = v + turn/2       反转：IN2 = 1, CCR = |speed|
                                                停转：|speed| ≤ 28
```

- 速度限幅 ±3600（100% 占空比），**死区 ±28** 防止堵转啸叫；
- `Motor::set_reversed(true)` 可在软件内翻转单侧转向，无需改接线。

### 当前固件：写死前进

`Core/Src/main.c` 处于调试模式，已注释掉遥控逻辑，上电后四轮满速前进：

```c
#define FORWARD_SPEED 3600.0f   // -3600 ~ 3600，3600 = 满占空比

while (1) {
    update(FORWARD_SPEED, 0.0f);
    HAL_Delay(30);
}
```

按比例调小 `FORWARD_SPEED` 即可降速；低于 ±28 会落入死区。恢复遥控逻辑从 git 历史取回 PS2 控制分支（`git log -p Core/Src/main.c`）。

PS2 协议帧、按键位映射与引脚阻塞点见文档 [03](docs/03-PS2手柄遥控.md)。

---

## 构建

依赖 [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)（含 arm-none-eabi-gcc、CMake、Ninja）。

```bash
export PATH="/e/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin:/e/STM32CubeCLT_1.18.0/Ninja/bin:/e/STM32CubeCLT_1.18.0/CMake/bin:$PATH"

cmake -S . -B build/Debug -G Ninja \
      -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

cmake --build build/Debug
```

产物 `build/Debug/CRTC_CAR_DEMO.elf`，当前占用 RAM 1816 B (8.87%) / FLASH 10696 B (16.32%)。

> 构建目录与工具链强绑定：若被 IDE 自带的 MinGW 配置过，删掉 `build/Debug` 重新 configure，不要修补缓存。

---

## 烧录

双击 `flash.bat`（调用 STM32CubeProgrammer CLI，自动校验并判定结果），或手动执行：

```bash
cd "E:/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin"
./STM32_Programmer_CLI.exe -c port=SWD mode=UR \
  -w "path/to/CRTC_CAR_DEMO.elf" -v -rst
```

**排障**：正常连接会打印 `Voltage : 3.2V` 左右；若显示 `0.00V` / `1.x V` 且报 `Unable to get core ID`，说明目标板未正常供电或 SWD 未共地，先查供电与 GND 导通（详见文档 [04](docs/04-开发日志与问题排查.md)）。

> **重要**：`MX_TIM2_Init` / `MX_TIM3_Init` / `MX_SPI1_Init` 已停用，电机初始化改由 `chassis_init()` 完成。`CRTC_CAR_DEMO.ioc` 仍是旧配置，**不要用 CubeMX 重新生成代码**，否则会重新占用 PA6/PA7/PB0/PB1 并覆盖引脚修正。

---

## 已知问题

1. **PS2 驱动未适配**：`PS2.cpp` 仍按 SPI1 重映射走 PB3~PB6（板上为空脚）。板上 `CLK=PA7 / CMD=PA5` 与 SPI1 的 `SCK/MOSI` 互换、`DI=PA4` 落在 NSS 上，**硬件 SPI 不可用**，需改为软件模拟时序。
2. **右侧电机 O1/O2 顺序存疑**：H2/H4 排针的 6/7 脚与左侧相反，1:1 排线会让右侧反转。软件用 `Motor::set_reversed(true)` 一行处理，先实测再定。
3. **手柄掉线无保护**：`PS2::read()` 恒返回 `true`，`online_` 从未赋值，掉线后小车保持最后速度。
4. **缺少看门狗**：调试模式 `HAL_Delay(30)` 约 33 Hz 刷新，实际控速应改为定时器中断。
