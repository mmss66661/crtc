# CRTC_CAR_DEMO

基于 **STM32F103C8T6** 的 PS2 手柄遥控小车，使用 HAL 库 + CMake/Ninja 构建，C/C++ 混合编程。

PS2 手柄通过 SPI 读取摇杆与按键，主循环解算后驱动 4 路电机 PWM，实现前进、后退与差速转向。

---

## 硬件

| 项目 | 参数 |
| --- | --- |
| MCU | STM32F103C8T6（Cortex-M3，64 KB Flash / 20 KB SRAM） |
| 系统时钟 | HSE 8 MHz × PLL9 = **72 MHz**（AHB 72 / APB1 36 / APB2 72） |
| 手柄接口 | SPI1（**重映射**），主机模式，CPOL=1 / CPHA=2Edge，256 分频 |
| 电机驱动 | TIM2 + TIM3 共 8 路 PWM，20 kHz（72 MHz / 3600） |
| 调试接口 | SWD（PA13 / PA14） |

### 引脚分配

| 功能 | 引脚 | 说明 |
| --- | --- | --- |
| SPI1_SCK | PB3 | 重映射 |
| SPI1_MISO | PB4 | 重映射 |
| SPI1_MOSI | PB5 | 重映射 |
| PS2_CS | PB6 | GPIO 推挽输出，软件片选 |
| MOTOR1 PWM | PA0 / PA1 | TIM2_CH1 / CH2 |
| MOTOR2 PWM | PA2 / PA3 | TIM2_CH3 / CH4 |
| MOTOR3 PWM | PA6 / PA7 | TIM3_CH1 / CH2 |
| MOTOR4 PWM | PB0 / PB1 | TIM3_CH3 / CH4 |
| 方向/使能 | PA11, PA12, PB10, PB11 | GPIO 推挽输出 |

---

## 软件结构

```
Core/               STM32CubeMX 生成的 HAL 初始化（时钟、GPIO、TIM、SPI、中断）
Chassis/            底盘层
  Inc/Chassis.h       Motor 类（双 PWM 正反转）+ ChassisCtrl（差速解算）
  Src/Chassis.cpp     电机 PWM 启动、速度限幅、左右轮分配
PS2/                PS2 手柄驱动
  Inc/PS2.h           PS2 类 + C 接口（PS2_Init / PS2_ReadRaw）
  Src/PS2.cpp         SPI 9 字节帧协议、DWT 微秒延时
cmake/              CubeMX 生成的 CMake 目标（stm32cubemx / STM32_Drivers）
```

### 控制流程

```
PS2 手柄 ──SPI(9 字节帧)──> PS2::read() ──> PS2RawData
                                              │
                                   按键位解包 + 摇杆居中(减 128)
                                              │
                                              ▼
                            update(-ly, lx)  ──> ChassisCtrl::update()
                                              │
                              left  = v - turn/2
                              right = v + turn/2
                                              │
                                              ▼
                        Motor::set_speed() ──> TIMx->CCR 占空比
```

- 左摇杆 **Y 轴**（`ly`）控制前进后退，取反后作为线速度 `v`
- 左摇杆 **X 轴**（`lx`）控制转向，作为角速度 `turn`
- 速度限幅 ±3600（对应 100% 占空比），**死区 ±28** 防止电机堵转啸叫

### 当前固件：写死持续前进

当前 `Core/Src/main.c` 处于**调试模式**，已注释掉 PS2 遥控逻辑，上电后四轮持续前进：

```c
#define FORWARD_SPEED 1500.0f   // -3600 ~ 3600，1500 约 42% 占空比

while (1) {
    update(FORWARD_SPEED, 0.0f);   // 线速度 1500，转向 0
    HAL_Delay(30);
}
```

调整 `FORWARD_SPEED` 即可：
- 转速不够/地面打滑 → 调大（上限 3600）
- 太快 → 调小；低于 ±28 会落入死区，电机不转

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
RAM:   1896 B / 20 KB   (9.26%)
FLASH: 12836 B / 64 KB  (19.59%)
```

> 当前为「写死前进」调试固件，PS2 驱动未被引用，已被 `--gc-sections` 裁掉，故 FLASH 占用比遥控版本小。

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

1. ~~`ChassisCtrl` 构造函数数组越界~~ — 已修复（原 `_m[4]` 改为 `_m[3]`）。修复后四路电机均正确绑定 PWM 通道，`update()` 不会再解引用空指针。
2. **手柄掉线无保护**：`PS2::read()` 恒返回 `true`，`online_` 成员从未被赋值，手柄拔掉后小车会保持最后一次速度继续跑。（当前固件未使用手柄，恢复遥控时需一并处理）
3. `PS2::transfer()` 中 `HAL_SPI_TransmitReceive` 的返回值未检查（`-Wunused-variable`）。
4. `Motor` 构造函数的成员初始化列表顺序与声明顺序不一致（`-Wreorder`）。
5. 缺少看门狗；调试模式 `HAL_Delay(30)` 约 33 Hz 刷新，实际控速建议改为定时器中断。

