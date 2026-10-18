# CRTC_CAR_DEMO

基于 **STM32F103C8T6** 的 PS2 手柄遥控小车，参加**重庆大学机器人训练大赛 CRTC2026**。
使用 HAL 库 + CMake/Ninja 构建，C/C++ 混合编程。

PS2 手柄通过 SPI 读取摇杆与按键，主循环解算后驱动 4 路电机 PWM，实现前进、后退与左右横移等全向移动。

---

## 仓库结构

```
.
├── README.md                     本文件：项目概览与开发指引
├── docs/                         【参赛文档】按 CRTC2026 手册 4.3 规范存放
│   ├── README.md                 文档索引 + 中期考核指标完成情况
│   ├── 01-整体方案与硬件设计.md
│   ├── 02-底盘运动.md            含演示视频
│   ├── 03-PS2手柄遥控.md
│   ├── 04-开发日志与问题排查.md
│   └── media/
│       └── chassis-motion.mp4    底盘运动演示视频
├── Chassis/                      底盘层：电机控制 + 麦轮解算 + 板级初始化
├── PS2/                          PS2 手柄驱动
├── Core/                         CubeMX 生成的 HAL 初始化
├── Drivers/                      STM32 HAL / CMSIS
├── cmake/                        构建配置
├── CMakeLists.txt                构建入口
├── flash.bat                     一键烧录脚本（ST-LINK）
└── build/Debug/                  构建产物（已 gitignore）
```

> 参赛文档全部位于 `docs/` 目录，详细设计说明与技术决策记录请从那里开始阅读。

---

## 硬件

| 项目 | 参数 |
| --- | --- |
| MCU | STM32F103C8T6（Cortex-M3，64 KB Flash / 20 KB SRAM） |
| 系统时钟 | HSE 8 MHz × PLL9 = **72 MHz**（AHB 72 / APB1 36 / APB2 72） |
| 手柄接口 | PS2 接收器，接 PA4~PA7（当前固件未启用） |
| 电机驱动 | TIM1 CH1~CH4 输出 PWM，20 kHz（72 MHz / 3600），IN2 与 STBY 由 GPIO 控制 |
| 调试接口 | SWD（PA13 / PA14） |

### 引脚分配

引脚以原理图（`CRTC_CAR_DEMO.ioc` 之外的 PCB 设计）为准：

| MCU 引脚 | 网络名 | 功能 |
| --- | --- | --- |
| PA8 / PB12 | FL_IN1 / FL_IN2 | 左前电机（CH1 + GPIO） |
| PA9 / PB13 | FR_IN1 / FR_IN2 | 右前电机（CH2 + GPIO） |
| PA10 / PB14 | BL_IN1 / BL_IN2 | 左后电机（CH3 + GPIO） |
| PA11 / PB15 | BR_IN1 / BR_IN2 | 右后电机（CH4 + GPIO） |
| PB0 | STBY1 | 前级驱动使能（高有效） |
| PB1 | STBY2 | 后级驱动使能（高有效） |
| PA7 | CLK | PS2 时钟 |
| PA6 | CS | PS2 片选 |
| PA5 | CMD | PS2 命令（MCU → 手柄） |
| PA4 | DI | PS2 数据（手柄 → MCU） |

> 板上 PB3 / PB4 / PB5 / PB6 为空脚，`MX_TIM2_Init`、`MX_TIM3_Init`、`MX_SPI1_Init` 已停用。


---

## 软件结构

```
Core/               STM32CubeMX 生成的 HAL 初始化（时钟、GPIO、中断）
Chassis/            底盘层（含板级引脚初始化，不再依赖 tim.c / spi.c）
  Inc/Chassis.h       Motor 类（PWM + 方向脚）+ ChassisCtrl（麦轮解算）
  Src/Chassis.cpp     TIM1 PWM 初始化、STBY 使能、麦轮逆运动学解算
PS2/                PS2 手柄驱动（当前固件未启用，引脚尚未适配）
  Inc/PS2.h           PS2 类 + C 接口（PS2_Init / PS2_ReadRaw）
  Src/PS2.cpp         SPI 9 字节帧协议、DWT 微秒延时
cmake/              CubeMX 生成的 CMake 目标（stm32cubemx / STM32_Drivers）
```

### 控制流程

```
chassis_move(vx, vy, w)  ──> ChassisCtrl::update_mecanum()
                                  │
        FL = vx - vy - w      FR = vx + vy + w
        BL = vx + vy - w      BR = vx - vy + w
                                  │
                          超量程时等比缩放
                                  │
                                  ▼
                         Motor::set_speed()
                                  │
              ┌───────────────────┴───────────────────┐
         正转 │ IN2 = 低, CCR = |speed|   反转 │ IN2 = 高, CCR = |speed|
              └───────────────────┬───────────────────┘
                                  ▼
                        TIM1 CHx ──> IN1 占空比
```

- `vx` 正为前进，`vy` 正为右移，`w` 正为逆时针自转
- 速度限幅 ±3600（对应 100% 占空比），**死区 ±28** 防止电机堵转啸叫
- `Motor::set_reversed(true)` 可在软件里翻转单侧转向，无需改接线

### 当前固件：麦轮四方向轮播演示

`Core/Src/main.c` 处于**调试模式**（PS2 遥控暂未启用），上电后按 **前 → 后 → 左 → 右** 固定顺序循环移动，序列写死不随机：

```c
#define MOVE_SPEED    3600.0f   // 轮速，满占空比
#define MOVE_TIME_MS  3000U     // 每个方向持续 3 秒
#define MOVE_GAP_MS   5000U     // 动作之间停顿 5 秒

static const float kDirVx[4] = {1.0f, -1.0f, 0.0f, 0.0f};   // 前 后 左 右
static const float kDirVy[4] = {0.0f, 0.0f, -1.0f, 1.0f};

while (1) {
    chassis_move(MOVE_SPEED * kDirVx[step], MOVE_SPEED * kDirVy[step], 0.0f);
    HAL_Delay(MOVE_TIME_MS);
    chassis_move(0.0f, 0.0f, 0.0f);
    HAL_Delay(MOVE_GAP_MS);
    step = (step + 1U) % 4U;
}
```

一轮完整循环 32 秒（每个方向 3 s + 停顿 5 s）。改 `MOVE_TIME_MS` / `MOVE_GAP_MS` 即可调整节奏。

需要恢复手柄遥控时，从 git 历史取回 PS2 控制分支即可（`git log -p Core/Src/main.c`）。

### PS2 按键位映射

手柄数据低有效，代码中先取反再判断：

| bit | 按键 | bit | 按键 |
| --- | --- | --- | --- |
| 4 | ↑ | 12 | Y（△） |
| 5 | → | 13 | B（○） |
| 6 | ↓ | 14 | A（×） |
| 7 | ← | 15 | X（□） |
| 8 | L2 | | |
| 9 | R2 | | |
| 10 | L1 | | |
| 11 | R1 | | |

摇杆原始值 0~255，中心 128，代码中统一减去 128 转为 -128~127。

---

## 构建

依赖 [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)（内含 arm-none-eabi-gcc、CMake、Ninja）。

```bash
# 把工具链加入 PATH（按自己的安装路径调整）
export PATH="/e/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin:/e/STM32CubeCLT_1.18.0/Ninja/bin:/e/STM32CubeCLT_1.18.0/CMake/bin:$PATH"

# 配置
cmake -S . -B build/Debug -G Ninja \
      -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
      -DCMAKE_MAKE_PROGRAM="E:/STM32CubeCLT_1.18.0/Ninja/bin/ninja.exe"

# 编译
cmake --build build/Debug
```

Windows PowerShell 下等价写法：

```powershell
$env:PATH = "E:\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin;E:\STM32CubeCLT_1.18.0\Ninja\bin;E:\STM32CubeCLT_1.18.0\CMake\bin;$env:PATH"
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug
```

产物：`build/Debug/CRTC_CAR_DEMO.elf`

当前资源占用：

```
RAM:   1816 B / 20 KB   (8.87%)
FLASH: 10696 B / 64 KB  (16.32%)
```

> 当前为「写死前进」调试固件，PS2 驱动未被引用，已被 `--gc-sections` 裁掉，故 FLASH 占用比遥控版本小。

> **重要**：`MX_TIM2_Init` / `MX_TIM3_Init` / `MX_SPI1_Init` 已在 `main.c` 中停用，电机初始化改由 `chassis_init()` 完成。`CRTC_CAR_DEMO.ioc` 仍是旧配置，**不要用 CubeMX 重新生成代码**，否则会把 PA6/PA7/PB0/PB1 重新配成 TIM3 PWM 并覆盖本目录的引脚修正。

> 注意：构建缓存与工具链强绑定。若之前用 CLion 自带的 MinGW 配置过 `build/Debug`，`CMakeCache.txt` 中会残留 MSVC/MinGW 路径导致构建失败，删掉 `build/Debug` 重新配置即可。

---

## 烧录

### 一键脚本（推荐）

双击项目根目录的 `flash.bat`，脚本会调用 STM32CubeProgrammer CLI 下载并校验，自动判定成功/失败。

### 手动命令

```bash
cd "E:/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin"
./STM32_Programmer_CLI.exe -c port=SWD mode=UR \
  -w "path/to/CRTC_CAR_DEMO.elf" -v -rst
```

- `mode=UR`：connect under reset，最稳的连接方式
- `-v`：下载后校验
- `-rst`：烧完复位运行

**排障**：正常连接时 CLI 会打印 `Voltage : 3.2V` 左右。若显示 `0.00V` 或 `1.x V` 且报 `Unable to get core ID`，说明目标板没有正常供电或 SWD 地线未连通——先用万用表确认 ST-LINK 的 GND 与板子 GND 导通、板子 VDD 有稳定 3.3V。STM32F103 最低工作电压为 2.0 V，低于此值内核不会启动。

---

## 已知问题 / TODO

1. ~~`ChassisCtrl` 构造函数数组越界~~ — 已修复（原 `_m[4]` 改为 `_m[3]`）。
2. ~~引脚与 PCB 不匹配~~ — 已修复。原固件用 TIM2(PA0~PA3)/TIM3(PA6,PA7,PB0,PB1)/SPI1 重映射(PB3~PB6)，而板上这些脚是空脚、PS2 的 CS/CLK 和 STBY。现改为 **TIM1 CH1~CH4 (PA8~PA11) + PB12~PB15 方向 + PB0/PB1 使能**。
3. **PS2 驱动尚未适配**：`PS2.cpp` 仍按 SPI1 重映射走 PB3~PB6，与板上 PA4~PA7 不符。而且板上 `CLK=PA7 / CMD=PA5` 与 SPI1 硬件功能（`PA5=SCK`、`PA7=MOSI`）正好互换，`DI=PA4` 落在 NSS 上——**硬件 SPI 用不了，需要改成软件模拟时序（bit-bang）**。
4. **右侧电机 O1/O2 顺序存疑**：H2/H4 排针上 `FR_O2/FR_O1`（及 `BR_O2/BR_O1`）与左侧相反，若驱动板排线 1:1 直通，右前/右后轮会反转。软件里用 `Motor::set_reversed(true)` 一行即可翻转，先实测再定。
5. `PS2::read()` 恒返回 `true`，`online_` 从未赋值 → 手柄掉线后小车会保持最后速度（恢复遥控时需一并处理）。
6. 缺少看门狗；调试模式 `HAL_Delay(30)` 约 33 Hz 刷新，实际控速建议改为定时器中断。


