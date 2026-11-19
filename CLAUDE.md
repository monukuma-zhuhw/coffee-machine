# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bare-metal (no RTOS) espresso-machine firmware for an **STM32F030C8Tx** (Cortex-M0, 48 MHz, 64 KB flash, 8 KB SRAM), written against the **STM32F0 Standard Peripheral Library (SPL, not HAL)**. It runs a PID-controlled boiler, a relay-controlled brew-head heater, a steam heater, an AC vibration pump driven by frequency synthesis, brew/hot-water valves, and a DWIN DGUS II touchscreen. Customer settings and 9 editable pump-curve presets are persisted in main flash.

`软件方案文档.md` is a Chinese design document — useful background, but several parts are out of date (see Known Issues). When it, a code comment and the code disagree, trust the code.

## Build / Flash / Debug

- **Toolchain**: Keil MDK uVision 5, ARMCC V5.06, project `project/NFC.uvprojx` (C99 on, defines `USE_STDPERIPH_DRIVER STM32F0XX`). Build in the IDE (Project → Build); there is no CLI build, makefile, lint config, CI or test suite, so there is no "run a single test" command — verify on hardware (see Testing below). Flash with J-Link.
- **Output**: `project/Objects/coffee.hex` (the target's OutputName is `coffee`; older docs say `NFC-bootlader`). Map file: `project/Listings/coffee.map`.
- **Do not edit `project/NFC.uvprojx`** — edits by Claude have corrupted it before. When you add/remove/move a `.c` file, tell the user to add it in Keil and give them the block, e.g. after `<FileName>heater_control.c</FileName>`'s `</File>`:
  ```
              <File>
                <FileName>foo.c</FileName>
                <FileType>1</FileType>
                <FilePath>..\HARDWARE\src\foo.c</FilePath>
              </File>
  ```
  New headers in `HARDWARE/inc` need nothing (already on the include path).
- **IROM1 must stay `0x08000000`, size `0xB000`** (Target options). Flash above `0x0800B000` is data (presets, then the settings page). `Preset_Init()` prints `[Preset] FATAL` and refuses to save presets if the linked image (+1 KB margin) reaches `0x0800B000` — code that no longer fits ~43 KB is a layout problem, not a reason to enlarge IROM1.
- **Testing = hardware + serial log.** Debug UART is USART1 PA9/PA10, 115200 8N1 (`printf`). With `SYS_UART_CMD_ENABLE` (sys.h, on) it also accepts line commands (`cmd_console.c`, optional leading `/`):
  `/extract` (= KEY1 brew) · `/hot` (= KEY2) · `/steam` (= KEY3) · `/cold` (hot valve + pump 100 % for 30 s, heaters off) · `/curve` (enter page 4 and start a variable-pressure brew) · `/help`.
  To add a command: one row in `s_cmd_table[]` plus a `Cmd_Trigger_*()` in `main.c`.
- **Debug switches**: `DEBUG_PRINT_VP_READ` (sys.h, off) logs every received 0x83 frame; `CURVE_DIAG_ENABLE` (main.c, 0) logs target vs. actual gear per curve sample; `UART_SWAP` (sys.h, off) swaps the debug and DWIN UARTs (`stm32f0xx_it.c` and the command console follow it).
- **Freeze vs. fault**: `HardFault_Handler` is a bare `while (1)` (prints nothing). If the serial log simply stops, halt with J-Link and look at the PC — a USART ISR re-entering forever points at an uncleared ORE (gotcha 7), the HardFault vector at a fault.
- **Tools**:
  - `tools/gen_preset_defaults.py` regenerates `HARDWARE/src/preset_defaults.c` (edit `PROFILES`, then `python tools/gen_preset_defaults.py`).
  - `tools/serial_tool/` — Windows/Python serial monitor (`run.bat`, or `pip install -r requirements.txt` + `python serial_tool.py`) that plots `temp:<v>` / `pressure:<v>` tokens (case-insensitive). The firmware's 1 s summary line prints `Temp:` (plotted) but `Pres:` (not matched), so print `pressure:<v>` if you need the pressure trace.

## Source Layout

- `project/main.c` — boot sequence, main loop, all screen (VP) handling, beverage flows, curve page, console command handlers.
- `project/stm32f0xx_it.c` — ISRs: SysTick (`g_tick` + boiler heater soft-PWM), USART1/USART2 (DWIN parser vs. command console per `UART_SWAP`), EXTI4_15 (encoder), TIM14 (pump), EXTI0_1 (empty ZCD stub).
- `HARDWARE/inc|src` — one module per peripheral/feature (`.h` = pins/tunables/API, `.c` = static state); most are covered under Architecture below. The less obvious ones:
  - `pid_controller` — generic PID used by `boiler.c`'s state machine.
  - `heater_control` — PA12 software PWM for the boiler heater + over-temperature protection.
  - `steam` (PB12 pump + PB13 valve relays) vs. `steam_heater` (PA11, TIM1_CH4 PWM) — two modules for one steam flow.
  - `gpio_ctrl` — legacy POWER_CH1 output (PA5), only used once at boot via `GPIO_CtrlInitAll`.
  - `temperature_sensor` (boiler NTC, PA4), `outlet_temp_sensor` (brew-head NTC, PA0), `pressure_sensor` (PA6), `water_level` (PC13) — thin wrappers; the three ADC ones share ADC1.
- `SYSTEM/` — BSP: `sys.h` (type aliases, UART pin selection, feature switches), `delay` (`g_tick`/`GetTick()`/`delay_ms`), `usart` (debug UART/printf).
- `Library/` — SPL + CMSIS, do not modify.
- `文本映射表.txt` — screen-side map of text descriptor pointers (SPs) used by `language.c`.
- `docs/dwin_screen.md` — DWIN protocol/VP map, curve display and language details (split out of this file).
- `.claudeignore` lists what needs no reading: `Library/`, `project/Objects` and `project/Lists`, `*.hex` / `*.map` / `*.axf`, `tools/serial_tool/`, `dgus/`, `文本映射表.txt` and screen binaries (`*.hmi`, `*.tft`). Open them explicitly when you actually need them.

## Architecture

### Main loop (main.c)

Single non-blocking super-loop, all timing from `GetTick()` (1 ms). Every iteration, in order: `DWIN_ProcessRecvFrame()` → `SelPoll_Run` (0xD100–0xD118 → pages 5–11) → `D218_Curve_Poll` → `D04C_Light_Poll` → `SettingsVP_Poll` → `Language_Poll(now)` → preset factory-restore task → curve-page exit / save-dialog handling → `CooldownTest_Poll` (0xD000) → `CmdConsole_Poll` → key events → encoder delta. Then:

| Period | What runs |
|---|---|
| 10 ms (`FAST_LOOP_PERIOD_MS`) | boiler NTC + brew-head NTC sampling, `KeyMonitor_Update`, `HEATER_CheckProtection` (over-temp), `Boiler_Update`, `BrewHeater_Update`, `Beverage_Update` |
| 250 ms (`g_curve_sample_period`) | curve-page sampling, only while `g_curve_running` |
| 1000 ms | screen values (0xD004 boiler temp, 0xD008 pressure, 0xD010 head temp — floats, temps quantized to 0.5 ℃; 0xD00C brew seconds), the `Temp: … Outlet: … State: …` log line, `Settings_Task` |

MCU-initiated page changes go through `Page_SwitchTo()` and are followed by a blocking `delay_ms(300)` everywhere (see gotcha 5).

### Boot sequence (order matters)

1. `delay_init` / `uart_init` / `DWIN_UART_Init` → `Settings_Init` → `GPIO_CtrlInitAll` (drives PA5 as the old POWER_CH1 output; `KeyMonitor_Init` later re-claims it as KEY1) → `LightPWM_Init` + saved light level.
2. Sensors, heaters, pump, `CURVE_Init`, `Preset_Init` + `Preset_Load(curve_knob_val)`, `Encoder_Init`, `Boiler_Init`.
3. **Warm-hold starts immediately**: `Boiler_SetTarget(brew_temp - WARMUP_TEMP_OFFSET)` (offset 0) + `Boiler_HeatStart()` → `BOILER_HEAT_HOLD`; `BrewHeater_HeatStart(head_temp)`.
4. `Steam_Init`, `KeyMonitor_Init` (starts the 100 ms quiet window).
5. Page 0, register `on_vpread`/`on_touch`, wait 2 s, poll 0xA000 until it is stable 3× (boot animation done).
6. Write every initial VP from settings (displays, light, backlight, 0xD014/40/44/48/60, 0xD058/5C/70, pre-infusion texts 0xEA00/40/80, 0xD250) → page 1.
7. `SelPoll_Init` → `Language_Poll(GetTick())` (sends the first SP read) → main loop.

### Boiler (`boiler.c`) — PA12 heater + both outlet valves

States `IDLE`, `COOLING`, `HEATING`, `HOLDING`, `HEAT_HOLD`.

- **`HEAT_HOLD` is the normal standby and brewing state**: it only drives the heater (100 % below `BOILER_PREHEAT_TEMP` = 60 ℃, PID above) and never touches the pump or valves, so beverage flows can drive those.
- **`IDLE` actively forces heater, pump and brew valve off every 10 ms**; an invalid temperature (< −100) does the same in any state. `Boiler_SetExternalControl(1)` makes `Boiler_Update` return immediately (only `/cold` uses it).
- `Boiler_HeatStart()` closes both valves, stops the pump and resets the PID — call it **before** opening a valve / starting the pump, never after (gotcha 1).
- Screen 0xD000 click → `Boiler_StartCycle()`: COOLING (heater off, brew valve open, pump 50 Hz while water is present, until < `BOILER_COOL_TEMP` 70 ℃) → HEATING → HOLDING (valve open, pump curve 10→30→50 Hz over 33 s, PID compensates, 120 s) → **IDLE with the heater off**; warm-hold only resumes after the next `Beverage_Stop()`.
- PID (`Boiler_PIDStep`): gain-scheduled by |error| as a % of target (FAR/MID/NEAR gains, output caps per error band, integral deadband, overheat hard-off band, overshoot-prediction feed-forward). All tunables are in `boiler.h`; percentages are used because the target is adjustable.
- Heater: PA12 software PWM, 10 Hz / 100 steps from SysTick (`HEATER_PWM_Tick`), **low = heat** (PA12 has no timer AF on F030).
- **Over-temperature protection** (`heater_control.c`):
  - `HEATER_CheckProtection(temp)` runs every 10 ms, before `Boiler_Update`.
  - At ≥ `HEATER_PROTECT_TEMP` (120 ℃) it cuts the heater (`>>> Boiler over-temp …`), and `HEATER_SetPower()` then accepts only 0 until the reading drops below `HEATER_PROTECT_RELEASE_TEMP` (115 ℃).
  - It only blocks the boiler heater. The pump, the steam heater and the brew-head heater are untouched.
- Valves: PB2 brew, PB11 hot water, high = open, interlocked (opening one closes the other).

### Brew-head heater (`brew_heater.c`)

PB10 relay (**high = heat**; PB10 has no timer AF) holds the group-head block — measured by the PA0 NTC (`OTEMP`, shown at 0xD010) — between `head_temp - BREW_HYST_LOW_C` (3) and `head_temp - BREW_HYST_HIGH_C` (0), stop decisions throttled to 1 s. Enabled at boot and never stopped by `Beverage_Stop`; target = `settings.head_temp` (VP 0xD044). Independent of the boiler loop.

### Beverage flows (`main.c`, `Bev_State_t`)

Keys (press = low; every press beeps via `DWIN_BuzzerPlayOnce`): while a flow runs, **any key stops it**; otherwise KEY1/PA5 = brew (variable-pressure brew when on page 4), KEY2/PA7 = hot water, KEY3/PB7 = steam. A start is refused unless `g_bev_state == BEV_IDLE`, the boiler is IDLE or HEAT_HOLD, and the tank has water.

- **Brew** (`Beverage_StartBrewEx`): retargets the boiler to `brew_temp` but **does not wait for temperature** (`BEV_BREW_HEAT` is dormant), jumps to page 3. With `preinfuse_master`: PREINFUSE (brew valve + pump 25 % = 12.5 Hz for `preinfuse_duration_s`) → PREINFUSE_STOP (pump/valve off for `preinfuse_stop_s`) → RUN; otherwise straight to RUN. RUN = pump 100 % (50 Hz) open loop for `brew_time_s` (curve-page brews have their own length rule — see Variable-Pressure Mode); the cup counter increments once RUN has lasted 15 s.
- **Hot water**: PB11 + pump 50 Hz for `hot_time_s`; the boiler stays in warm-hold.
- **Steam**: PB13 steam valve + PB12 steam pump + PA11 steam heater all on together, max 3 min; the boiler is not touched.
- Every flow also stops on its timeout or when `WATER_IsPresent()` goes false.
- `Beverage_Stop()`: everything off, then **restores warm-hold** (`SetTarget(brew_temp)` + `HeatStart`). Only brew flows return to page 1, and not when started from the curve page.
- Units: `AC_Pump_SetPower()` takes 0.1 % (0–1000, linear 0–50 Hz); `AC_Pump_SetFrequency()` takes qhz = 0.25 Hz (0–200). Pressure is measured and displayed only — the pressure PID was removed (2026-10-25); `g_target_pressure` is a leftover display/log value.

### AC pump (`ac_pump.c`)

The TIM14 1 ms update ISR synthesizes a 30 %-duty square wave on **PC14 + PC15** (mirrored, written together through BSRR/BRR), 0–50 Hz in 0.25 Hz steps. `AC_Pump_On()` resets the phase counter — only call it on an off→on transition (see Variable-Pressure Mode). The old TRIAC/zero-cross design is gone; `AC_Pump_ZCD_IRQHandler()` is an empty stub still wired to EXTI0_1.

### Sensors and inputs

- Boiler NTC PA4/CH4, brew-head NTC PA0/CH0, pressure PA6/CH6 (0.5–4.5 V → 0–16 bar, `P=(V-0.5)*4`, α=0.2 low-pass; `PRES_ReadRaw(&mv)` = unfiltered sample + its voltage). All share ADC1, and each read clears `ADC1->CHSELR` first because F0's `ADC_ChannelConfig` ORs channels in.
- Tank level PC13, pull-up, **low = water**; floating reads "empty", which inhibits pumping (fail-safe).
- Keys (`key_monitor.c`): pull-up, OFF = high / ON = low, 20 ms debounce, `KeyMonitor_PopEvent()` queue, 100 ms boot quiet window (gotcha 6).
- Encoder PB9 (A) / PB8 (B), EXTI4_15, software quadrature table, 4 edges per detent, `Encoder_PopDelta()`. Used only on the curve page: before KEY1 it cycles the preset slot 0–8 (VP 0xD250, persisted); while recording it changes the pump frequency ±1.25 Hz per detent.

### Light board (`light_pwm.c`)

PB1 = TIM3_CH4 (AF1) hardware PWM, 1 kHz, level 0–10 → duty level × 10 %. Screen side: 0xD04C and 0xD050 are mirrored on/off switches (the MCU syncs the other one); 0xD054 is the level, saved to `settings.light_level` only while the switch is on; switching off forces 0xD054 = 0 on screen (the saved level is kept), switching on restores it. `D04C_Light_Poll` also polls 0xD04C every 500 ms.

### Settings (`settings.c`)

- Stored in the last 1 KB flash page `0x0800FC00`: magic `0xC0FF` + `SETTINGS_VERSION` (0x0008) + fields + u16 word-sum checksum. Blank, corrupt, out-of-range or **version-mismatched** data → defaults written back, so **bumping `SETTINGS_VERSION` resets every customer setting**. Add new fields before `reserved` and bump the version.
- Edit with `Settings_Edit()` + `Settings_MarkDirty()`; `Settings_Task()` (1 s loop) commits 1 s after the last change; `Settings_Save()` writes immediately.
- Screen sync: auto-upload frames plus `SettingsVP_Poll`, which issues one `DWIN_ReadVar` every 500 ms round-robin over 8 VPs — 0xD014 cups, 0xD040 brew temp (20–110), 0xD044 head temp (20–110), 0xD048 brew time (15–99 s), 0xD060 language, 0xD058 pre-infusion s (1–9), 0xD05C soak s (1–9), 0xD070 pre-infusion on/off. 0xD058/0xD05C are validated directly in `on_vpread`, the rest in `SettingsVP_Apply`. A new brew temp retargets the boiler immediately, a new head temp retargets the brew-head heater.
- 0xD22C 0→1 resets the cup counter. 0xD080 0→1 = factory reset: defaults + immediate save + resync of every VP and output + asynchronous rewrite of all 9 presets (`Preset_RestoreAllTask`, one page every `PRESET_RESTORE_GAP_MS` = 300 ms, only while `BEV_IDLE`).

### Preset Pump-Frequency Curves (9 slots x 400 points, gear units)

`HARDWARE/src/preset_store.c` + `preset_defaults.c` store **9 editable pump-frequency curves** in the main flash.

- **Data unit: gear 0..40** (0 = pump off), **1 gear = 1.25 Hz** (40 gear = 50 Hz). This is an exact multiple of the AC pump's native 0.25 Hz step (`qhz = gear * 5`), so no precision is lost. Curve Y axis = `gear * 12.5` (Hz x 10) -> 0..500.
- **Curve length** = index of the first `PRESET_END_MARK` (200) byte (`Preset_Length()`; factory curves have 396 points = 99 s, matching `CURVE_REC_MAX_POINTS`). A recording that follows the preset stays below that length. After a knob touch it may run past it, overwriting the old mark, up to `CURVE_REC_MAX_POINTS` = 396. On save, `Preset_SetLength()` writes the mark at the recorded length and recomputes the length from the first mark it finds.
- **Flash layout**: 9 consecutive 2 KB pages at `0x0800B000` (`PRESET_FLASH_BASE`), **one slot per page** - required because the F030 flash erase granularity is a whole 2 KB page, so two slots can never share a page. Each page holds only 408 programmed bytes: header (`magic 'PR'`, version, crc16, point count) + 400 gear bytes; the remaining 1640 bytes stay erased (0xFF) and are not programmed.
- **Customer settings** remain at `0x0800FC00` (page `0x0800F800`), untouched by preset writes.
- **RAM**: one 400-byte working buffer (`g_work`) holds the currently viewed/edited slot. `Preset_Load(slot)` reads it from flash; if the slot is invalid (blank / crc fail) it falls back to the factory curve **without** writing flash. `Preset_Save(slot)` erases the page, programs 408 bytes, and verifies by read-back.
- **Factory curves** are baked into the firmware as `g_preset_defaults[9][400]` (3600 bytes of constdata), generated by `tools/gen_preset_defaults.py` (edit the `PROFILES` table there and re-run). They are 9 variants of pre-infuse hold -> linear ramp -> peak hold.
- **Slot selection**: the curve-page knob `0xD250` (`settings.curve_knob_val`, 0-8) *is* the slot index. Turning it reloads the slot and redraws CH1.
- **Saving costs ~70 ms wall time but only ~30 ms with interrupts disabled** - see the "Flash Write Rules" section below; the erase and the program are deliberately split into two critical sections.
- **`Image$$ER_IROM1$$Limit` guard**: `Preset_Init()` checks that the linked image ends below `PRESET_FLASH_BASE` (plus 1 KB margin for the RW init image). If the Keil IROM1 size was not shrunk to `0xB000`, saving is refused and a FATAL message is printed - otherwise a preset write would erase code and brick the machine.
- **Wear**: one page erase per save, >=1000 erase cycles per page -> >=1000 saves per slot.

### Flash Write Rules (主 Flash 擦写 - 2026-10-26 死机修复)

`HARDWARE/src/flash_op.c` + `flash_op.h` 是唯一的 Flash 擦写底层 (`settings.c` / `preset_store.c` 都走它)。**改这个文件前先读完这一节。**

F0 单 Bank：擦写期间 CPU 不能从 Flash 取指，所以**擦写全程必须关中断** (`FLASH_OP_CRITICAL_BEGIN/END`)。关中断期间：

- 中断全部挂起 -> SysTick 停走 -> **泵/加热的时序冻结**；
- **串口接收缓冲区溢出** (F0 的 USART 只有 1 字节接收缓冲) -> 屏发来的帧丢字节 -> DGUS 帧的 `len` 字段变垃圾值 -> 解析器吞掉后面好几帧。

**因此关中断窗口的长度是有硬上限的（~30 ms），不是"能塞多少塞多少"。**

| 规则 | 原因 |
|---|---|
| **擦除和编程必须拆成两个临界区**，中间 `delay_ms(30)` | F0 擦完一页 2KB 后内部需要恢复时间才能接着写。擦完立刻接着写(哪怕只隔几毫秒)是"写一次成功 -> 紧接着再写就死"的触发条件。中间的 `delay_ms` 同时让 SysTick 跑起来，把泵/加热冻结时间砍到只剩擦除那 30 ms |
| **编程用 `Flash_ProgramBlock()` 整段连续写**，不要逐半字调 `Flash_ProgramHalfWord()` | 逐半字写 400 字节 = 204 次"清标志->置PG->写->等BSY->清PG"往返，串起来把关中断窗口撑到 ~200 ms。`Flash_ProgramBlock` 全程保持 PG 置位、每半字只等 BSY + 清一次 EOP，400 字节只要 ~10 ms |
| **绝对不要在临界区里调 `printf`** | `settings.c` 早就写了这句注释，`preset_store.c` 之前违反了它。printf 会把一大段库代码拉进临界区（栈占用也大）。诊断信息一律挪到临界区外打 |
| **不要用空转循环模拟"恢复延时"** | 之前加的 `Flash_BusyWait(60000)` 注释写 5 ms、实际 ~17 ms；`Flash_BusyWait(2000)` 注释写 160 us、实际 ~0.55 ms × 204 = +112 ms。就是它把"改配置也死机"引进来的。要延时用 `delay_ms()`（在临界区**外**） |
| **擦除地址可以不对齐页边界** | F0 的 FLASH_AR 只取高 11 位选页，`0x0800FC00`（设置页，实际在 `0x0800F800` 这页里）能正确擦整页 |

写完之后 `flash_op.c` 会做**回读校验**（擦完首字节必须是 0xFF；写完逐半字比对）。校验不过返回非 0 码：`1`=Flash 卡在忙 / `2`=超时 / `3`=PGERR/WRPERR / `4`=回读不一致。

### Variable-Pressure Mode (变压模式, curve page 4)

`main.c` drives the pump **from the preset curve** while recording, so the live curve (CH0) overlays the preset curve (CH1) exactly until the customer touches the knob.

- **Enter page 4** (`D218`) -> `Preset_Load(slot)` -> CH1 pre-filled with the preset (`CURVE_DrawRefFixed`, `mul_x100=1250`, repeat=1). Knob scope = slot select (0-8).
- **KEY1** -> `Beverage_CurveStartCH0()`: starts the normal brew flow (`Beverage_StartBrewEx`) **and** sets `g_curve_drive=1`. In this mode the brew **skips pre-infusion/soak** (the pump would be off during soak, so the live curve could not match the preset); the preset's early low-gear points *are* the pre-infusion. If `Beverage_StartBrewEx` refuses (boiler busy / tank empty), the recording is rolled back — otherwise the sampler would drive the pump with the valve closed.
- **Duration ignores `brew_time_s`** (`BEV_BREW_RUN` skips that timeout while `g_curve_drive && g_curve_running`). One point per 250 ms, capped at `CURVE_REC_MAX_POINTS` = 396 points (99 s).
  - Knob never touched: the brew follows the preset and ends at its `PRESET_END_MARK` (or at 396 points, whichever comes first). The sampler calls `Beverage_Stop()` and the screen stays on page 4.
  - Knob touched (the customer is controlling the pump): the preset length no longer applies, and recording continues past the preset's end mark up to 396 points. If the customer hasn't exited by then, the sampler sets `g_curve_exit_pending` (`>>> Curve: 396 pts (99s) reached, auto exit`), so the brew ends exactly like a 0xD300 exit, including the 15 s save-prompt rule below.
  - A key press, an empty tank, or leaving page 4 still stops it early.
- **Every 250 ms sample** (`g_curve_sample_period`): (1) if the knob has not been touched, `Curve_ApplyPresetPump(idx)` sets the pump to `preset[idx] * 5` qhz; (2) the **actual** frequency is converted back to a gear and **written into the working buffer** at that index; (3) CH0 gets the actual frequency. Read the preset *before* overwriting it.
- **`AC_Pump_On()` must not be called every sample**: it resets the PWM phase counter (`g_toggle_counter=0`), which would chop a 50 Hz square wave into 24 ms pulses and stop the pump. Only call it on the off->on transition.
- **Knob during recording**: sets the pump frequency manually (1.25 Hz/detent) and sets `g_curve_knob_touched=1` -> the live curve stops following the preset. The first touch also records `g_preset_edit_tick` (the 15 s reference point).
- **Manual exit** (screen reports touch VP `0xD300`; **any report counts as an exit, the value is ignored**; only honoured while on page 4): stop recording, stop the brew, then
  - not edited, or exited **within 15 s** of the first knob turn -> silently discard (reload the slot from flash) and go to page 1;
  - **more than 15 s** after the first knob turn -> switch to the save-prompt page (`CURVE_SAVE_PROMPT_PAGE`, default 12) with 保存/取消 buttons. 保存 -> `Preset_SetLength(g_curve_ch0_count)` + `Preset_Save(slot)`: the end mark moves to the end of the actual recording, so the saved curve is exactly what was recorded (shorter if stopped early, up to 396 points if the knob extended it); 取消 -> discard. Either way the MCU then returns to page 1.
  - **Page 12 dialog** (save prompt): 是 button reports `0xD30C` = 1 -> save; 否 button reports `0xD308` = 1 -> discard; `0xD304` (a second 否) reports any non-zero -> discard. Other values are ignored. On entering the dialog the MCU clears all three button VPs to 0 and resets the 0->1 edge detectors, so a stale screen value can never trigger a save by itself.
  - **Page 12 dialog texts** (added to `language.c` `g_text[]`, SP = text descriptor address): `0x8940` "新数据是否替换并保存?" / "Replace And Save New Data ?", `0x8980` "是" / "Yes", `0x89C0` "否" / "No". The Chinese characters 新/据/是/否/替/换/并/保/存 were added to the GBK table in `dwin_uart.c` (a missing entry silently drops the character, it does not error).
- The 15 s window is `CURVE_EDIT_PROMPT_DELAY_MS` in main.c.

### DWIN Screen (details: `docs/dwin_screen.md`)

T5L / DGUS II screen (DMG40960F040_01WTC), frames `5A A5 [len] [cmd] [data...]`, no checksum. The frame/command reference, the VP map, the RX-parser resync rules, screen-project and touch-file parsing, `curve_display.c` (dual-channel curves, X-scale rules) and `language.c` (text SP cache, CN/EN switching) are in `docs/dwin_screen.md` — read it before touching any of those.

Invariants that bite when you add or change a control:

- The USART ISR only assembles bytes into a 16-slot frame queue; `DWIN_ProcessRecvFrame()` (main loop) drains it and hands **every** 0x83 frame to `on_vpread(vp, value)` in main.c, which routes by VP address.
- **Momentary controls are edge-triggered**: the screen re-reports values, so act only on a 0→1 transition, and when the MCU writes a VP back to 0, reset its edge detector to 0 as well (otherwise the next report of 1 is swallowed).
- A text control's **data VP = its SP (descriptor) + 0x6000**; writing to the SP corrupts the descriptor. `DWIN_SetText()` converts UTF-8 to GBK through a table in `dwin_uart.c`; a character missing from the table is silently dropped.
- Keep `language.c` saved as UTF-8 with BOM (ARMCC must read the Chinese literals as UTF-8).

## Pin Mapping

| Pin | Function |
|---|---|
| PA0 | Brew-head NTC, ADC_CH0 (`OTEMP`; brew-head heater feedback, shown at 0xD010) |
| PA1 | Unused (old zero-cross input; EXTI0_1 still routes to an empty stub) |
| PA2 / PA3 | DWIN USART2 (USART1 with `UART_SWAP`) |
| PA4 | Boiler NTC, ADC_CH4 |
| PA5 | KEY1 brew — input pull-up (briefly the old POWER_CH1 output during `GPIO_CtrlInitAll`) |
| PA6 | Pressure sensor, ADC_CH6 |
| PA7 | KEY2 hot water — input pull-up |
| PA9 / PA10 | Debug USART1: printf + command console |
| PA11 | Steam heater, TIM1_CH4 (AF2), 1 kHz |
| PA12 | Boiler heater, software PWM 10 Hz, low = heat |
| PB1 | Light board, TIM3_CH4 (AF1), 1 kHz |
| PB2 | Brew valve, high = open (interlocked with PB11) |
| PB6 | Unused (former light board / power output) |
| PB7 | KEY3 steam — input pull-up |
| PB8 / PB9 | Encoder B / A (EXTI4_15) |
| PB10 | Brew-head heater relay, high = heat |
| PB11 | Hot-water valve, high = open (interlocked with PB2) |
| PB12 | Steam intake pump relay, high = on |
| PB13 | Steam output relay, high = on |
| PC13 | Tank level, pull-up, low = water |
| PC14 + PC15 | AC pump gate (mirrored outputs) |

## Timers, Interrupts, Conventions

- **Timers**: SysTick 1 ms = `g_tick` + boiler heater soft-PWM only · TIM1 = steam heater · TIM3 = light board · TIM14 = pump synthesis ISR · TIM15/16/17 free.
- **NVIC priorities**: SysTick = 0 (reset default, `delay_init` doesn't change it); DWIN USART and TIM14 = 1; encoder EXTI and debug USART = 2. Flash writes run with all interrupts disabled (see Flash Write Rules).
- **ISRs only buffer and set flags** (DWIN frames, console lines, encoder counts); all parsing and actions happen in the main loop.
- **Module pattern**: `X_Init()` configures hardware; `X_Update(..., now)` / `X_Poll(now)` is called from the main loop with `GetTick()`-based gates. Use unsigned `now - last >= period` comparisons (wrap-safe).
- `u8`/`u16`/`u32` and `PAout(n)`/`PAin(n)`-style macros come from `sys.h` (Cortex-M0 has no bit-band).

## Dormant / Removed

- `BEV_BREW_HEAT` (wait for temperature before brewing): dormant.
- Removed: pressure PID and knob → target pressure, PA15 confirm key, TRIAC/zero-cross pump control, the Sel2/Sel3 selection groups (0xD200/0xD204/0xD214), `LightBoard_Poll`/0xD208, the float pre-infusion VPs 0xD068/0xD06C and the float VP callback, PB6 software light PWM, `dwin_menu`/`dwin_icon_data`/`factory_menu`/`led`, the dormant `brewing_process` BREW state machine with its `g_pid` (its over-temp check now lives in `heater_control.c`, see Boiler).
- `文本映射表.txt` maps the English for 0x8780/0x87C0 wrongly (PRE SOAK/BREW TIME); `language.c` overrides them to BREW TEMP/HEAD TEMP.

## Known Bugs / Gotchas

These are real invariants — touch with care.

1. **Pump/valve flows must keep the boiler in `HEAT_HOLD`.** `Boiler_Update`'s IDLE branch turns the pump off and closes the brew valve every 10 ms, so any flow that drives them must call `Boiler_HeatStart()` *first* (it closes valves and stops the pump itself) and only then open the valve / start the pump — or use `Boiler_SetExternalControl(1)` as `/cold` does. Symptom when missed: `>>> Brew: … starting` is printed but the 1 s log shows `State: 0 … Valve: CLOSE`, pressure 0. `Beverage_StartHot()` calls `Boiler_HeatStart()` only when the boiler is not already in HEAT_HOLD (e.g. IDLE after a 0xD000 cycle — `Boiler_IsAvailableForBeverage()` accepts IDLE); during warm-hold it skips it on purpose, because the PID reset would drop heater power just as cold water flows in.
2. **Language SP reads used to hang forever on one lost reply.** Fixed with the 300 ms resend in `Language_Poll(now)` (`LANG_READ_TIMEOUT_MS`). Root cause (confirmed by serial log): `D04C_Light_Poll`'s `poll_tick` starts at 0, so on the first loop pass it immediately reads 0xD04C right next to the first `SP=0x8000` read, and the two replies raced for what was then a single-frame RX buffer (gotcha 3; now a queue, but replies can still be lost, so keep the resend). Diagnose with the `[Lang] INIT: send read SP=…` / `[Lang] SP=… read timeout, resend` / `[Lang] on_resp …` lines. **Don't** "fix" such races by reordering init calls or inserting `DWIN_ProcessRecvFrame()` drain loops at boot — that was tried and broke language switching outright.
3. **Screen frames can still be lost — but only in two cases.** Received frames go through a 16-slot queue (2026-10-26). Before that there was a single frame buffer, and any two frames arriving while the main loop was blocked overwrote each other. Typical blocking: the 1 s display refresh plus printf (the two 500 ms poll replies land right then), page-switch `delay_ms(300)`, and the ~65 ms settings commit. That buffer was why touches were lost "a second or two after editing settings". What is still lost: bytes arriving during a Flash erase (CPU stalled, ORE → frame dropped, gotcha 7), and frames that arrive after 16 are already queued. Anything waiting on a reply still needs a timeout/retry.
4. **`g_last_read_vp` is shared by all `DWIN_ReadVar` callers** and only reflects the latest request; routing is per VP in `on_vpread`, so don't build logic on it.
5. **Page switches block.** Every MCU page change is `Page_SwitchTo()` + `delay_ms(300)` (navigation, brew start, curve page, dialog, go-home). Don't add more blocking calls in poll helpers, and avoid page switches inside tight sequences of screen reads.
6. **KEY1 boot quiet window.** For 100 ms after `KeyMonitor_Init`, `KeyMonitor_Update` only syncs levels and emits no events: PA5 has just been switched from push-pull output (POWER_CH1) to input and the transient would otherwise register as a KEY1 press.
7. **USART ISRs must clear ORE with `USART_ClearFlag(USARTx, USART_FLAG_ORE)`.** On F0 only ICR.ORECF clears it (reading RDR doesn't), and with RXNEIE set an uncleared ORE re-raises the interrupt forever: the ISR sees RXNE=0, returns, re-enters, and the main loop starves — looks like a freeze while SysTick (priority 0) keeps running. Every Flash erase stalls the CPU 20–40 ms, so any byte the screen or the serial console sends meanwhile overruns. Both `DWIN_USART_IRQHandler` and `DebugUsart_RxByte` clear it (added 2026-10-26 for a factory-reset freeze); the DWIN ISR also drops the half-received frame.
8. **`g_on_curve_page` can go sticky.** It is set on entering page 4 and cleared by a 0xD300 report or by `Page_SwitchTo()` to another page. If the screen leaves page 4 by its own navigation without reporting 0xD300, KEY1 keeps starting variable-pressure brews — the screen project must report 0xD300 when leaving page 4.

## Known Issues (stale comments / docs)

Trust the code over these:
- `settings.h` says commits happen 5 s after the last edit; `SETTINGS_COMMIT_DELAY_MS` is 1000.
- `boiler.c`/`boiler.h` comments mention 40 ℃ (cool end) and 70/80 ℃ (preheat); the macros are `BOILER_COOL_TEMP` 70 ℃ and `BOILER_PREHEAT_TEMP` 60 ℃. The `boiler.h` file header also says the heater is "PA11 PWM" with a fixed 88 ℃ target (actual: PA12 software PWM, target from `settings.brew_temp`).
- `steam.h` (line ~8) says the steam heater is PA12 / TIM1_CH2 (actual: PA11 / TIM1_CH4, see `steam_heater`).
- `软件方案文档.md`: output `NFC-bootlader.hex` (actual `coffee.hex`), fixed boiler PID gains (now gain-scheduled), 100 Hz light PWM (actual 1 kHz), PB14/PB15 as SWD (F030 SWD is PA13/PA14).
