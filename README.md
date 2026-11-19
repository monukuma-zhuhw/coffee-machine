# coffee-machine

基于 **STM32F030C8Tx**（Cortex-M0，48 MHz，64 KB Flash，8 KB SRAM）的意式咖啡机固件。裸机（无 RTOS）超级循环架构，使用 STM32F0 标准外设库（SPL，非 HAL），人机界面为 **DWIN DGUS II 串口触摸屏**。

## 功能

- **锅炉温控**：PA12 软件 PWM 加热 + 分段增益 PID（按误差百分比调度）、过温保护（≥120 ℃ 切断，<115 ℃ 释放）
- **冲泡头加热**：PB10 继电器，NTC 滞环控温，独立于锅炉回路
- **蒸汽**：蒸汽泵、蒸汽阀、蒸汽加热器（TIM1_CH4 PWM）联动，最长 3 分钟
- **交流振动泵**：TIM14 中断合成 0–50 Hz 方波（步进 0.25 Hz），不依赖 TRIAC/过零检测
- **饮品流程**：萃取（可选预浸泡）、热水、蒸汽；任意按键可中断
- **变压模式（曲线页）**：按预设泵频曲线驱动，旋钮可实时改写并录制，保存为 9 个可编辑预设之一
- **预设曲线**：9 组 × 400 点（档位 0–40，1 档 = 1.25 Hz），存于主 Flash，附出厂默认值
- **设置持久化**：杯数、萃取温度、冲泡头温度、萃取时间、语言、预浸泡参数、灯光等，掉电保存
- **中英文切换**、补光灯 PWM 调光、水箱缺水保护、串口调试控制台

## 硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32F030C8Tx |
| 屏幕 | DWIN T5L / DGUS II（DMG40960F040_01WTC），帧格式 `5A A5 [len] [cmd] [data...]` |
| 传感器 | 锅炉 NTC（PA4）、冲泡头 NTC（PA0）、压力传感器（PA6，0.5–4.5 V → 0–16 bar）、水箱液位（PC13） |
| 输入 | 3 个按键（PA5 萃取 / PA7 热水 / PB7 蒸汽）、旋转编码器（PB8/PB9） |
| 调试串口 | USART1 PA9/PA10，115200 8N1 |

完整引脚表见 [CLAUDE.md](CLAUDE.md#pin-mapping)。

## 目录结构

```
project/            Keil 工程、main.c（启动流程/主循环/屏幕处理/饮品流程）、中断服务程序
HARDWARE/inc|src/   各外设/功能模块（boiler、brew_heater、ac_pump、steam、settings、preset_store、
                    flash_op、dwin_uart、language、curve_display、cmd_console 等）
SYSTEM/             BSP：sys.h（类型别名/功能开关）、delay、usart
Library/            STM32F0 SPL + CMSIS（请勿修改）
docs/dwin_screen.md DWIN 协议、VP 地址表、曲线显示与多语言说明
tools/              preset 默认曲线生成脚本、串口绘图监视工具
软件方案文档.md      中文设计文档（部分内容已过时，以代码为准）
文本映射表.txt       屏幕文本描述指针（SP）映射
CLAUDE.md           面向 AI 助手/开发者的详细架构说明与注意事项
```

## 构建与烧录

- 工具链：**Keil MDK uVision 5**，ARMCC V5.06，工程 `project/NFC.uvprojx`（C99，宏 `USE_STDPERIPH_DRIVER STM32F0XX`）
- 在 IDE 中 Build，使用 **J-Link** 烧录；无 Makefile / CI / 自动化测试
- 输出：`project/Objects/coffee.hex`，映射文件 `project/Listings/coffee.map`
- **IROM1 必须保持 `0x08000000`，大小 `0xB000`**：`0x0800B000` 之后为数据区（9 个预设页 + 设置页）。固件若超出该范围，`Preset_Init()` 会打印 `[Preset] FATAL` 并拒绝保存预设
- 新增 `.c` 文件需手动在 Keil 工程中添加

## Flash 布局

| 地址 | 用途 |
|---|---|
| `0x08000000`–`0x0800AFFF` | 程序代码（约 43 KB） |
| `0x0800B000` 起，9 × 2 KB | 9 个预设曲线（每槽一页） |
| `0x0800FC00` | 客户设置（所在页 `0x0800F800`） |

> 修改 Flash 擦写逻辑（`flash_op.c`）前，请先阅读 CLAUDE.md 中的 *Flash Write Rules*：擦除与编程须拆成两个关中断临界区，临界区内禁止 `printf`。

## 调试与测试

通过硬件 + 串口日志验证。启用 `SYS_UART_CMD_ENABLE`（`sys.h`，默认开）后，调试串口支持以下命令（可带前导 `/`）：

| 命令 | 作用 |
|---|---|
| `/extract` | 等同 KEY1，萃取 |
| `/hot` | 等同 KEY2，热水 |
| `/steam` | 等同 KEY3，蒸汽 |
| `/cold` | 热水阀 + 泵 100%，持续 30 s，加热器关闭 |
| `/curve` | 进入曲线页并开始变压萃取 |
| `/help` | 帮助 |

调试开关：`DEBUG_PRINT_VP_READ`、`CURVE_DIAG_ENABLE`、`UART_SWAP`。

## 工具

- `python tools/gen_preset_defaults.py`：编辑脚本中的 `PROFILES` 后，重新生成 `HARDWARE/src/preset_defaults.c`
- `tools/serial_tool/`：Windows/Python 串口监视器，实时绘制 `temp:<v>` / `pressure:<v>`。运行 `run.bat`，或 `pip install -r requirements.txt` 后 `python serial_tool.py`

## 开发注意事项

- 单一非阻塞超级循环，时基为 1 ms `GetTick()`；ISR 只缓冲数据/置标志，解析与动作在主循环完成
- 屏幕控件为**边沿触发**（0→1）；文本控件数据 VP = SP + 0x6000
- `language.c` 需保存为 **UTF-8 with BOM**
- 修改 `SETTINGS_VERSION` 会重置所有客户设置
- 更多已知问题（USART ORE、页面切换阻塞、泵/阀与锅炉状态联动等）见 [CLAUDE.md](CLAUDE.md) 的 *Known Bugs / Gotchas*

## 相关文档

- [CLAUDE.md](CLAUDE.md) — 架构、流程、陷阱的权威说明（以代码为准）
- [docs/dwin_screen.md](docs/dwin_screen.md) — 屏幕协议与 VP 地址表
- [软件方案文档.md](软件方案文档.md) — 设计背景（部分已过时）
