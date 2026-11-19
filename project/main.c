/**
 * @file        main.c
 * @brief       咖啡机控制器
 */
#include "stdio.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "dwin_uart.h"
#include "gpio_ctrl.h"
#include "temperature_sensor.h"
#include "pressure_sensor.h"
#include "curve_display.h"
#include "heater_control.h"
#include "brew_heater.h"
#include "ac_pump.h"
#include "encoder.h"
#include "boiler.h"
#include "steam.h"
#include "outlet_temp_sensor.h"
#include "water_level.h"
#include "steam_heater.h"
#include "key_monitor.h"
#include "settings.h"
#include "language.h"
#include "light_pwm.h"
#include "preset_store.h"
#if SYS_UART_CMD_ENABLE
#include "cmd_console.h"
#endif

#define ANIM_VP_ADDR     0xA000
#define BTN_VP_ADDR      0xD102
#define BTN_ICON_ID      0x01

/* 主循环快速节拍(ms): 温度采样/按键扫描/超温保护/锅炉PID/出饮流程 */
#define FAST_LOOP_PERIOD_MS  10

/* 目标压力默认值 (仅记录/显示, 无 PID 闭环) */
#define TARGET_PRES_DEFAULT  9     /* 开机默认目标压力(Bar) */

/* 曲线页 CH0 采样周期: 250ms/点 (步长2和步长4一致) */
#define CURVE_SAMPLE_PERIOD_MS  250

/* 实际曲线最长点数: 99s / 250ms = 396 点 (结束标志放在第 396 点, 预设区一组 400 点放得下).
 * 变压萃取最多录这么长; 客户扭旋钮手动控泵后不受预设长度限制, 录到这里自动退出萃取 */
#define CURVE_REC_MAX_POINTS    396u

/* 变压模式逐点诊断打印开关:
 * 0 = 关(默认, 每250ms一条太吵且拖慢主循环)
 * 1 = 开: 每采样点打印 "目标档位 vs 实际档位", 用来确认泵是否真的跟随预设曲线 */
#define CURVE_DIAG_ENABLE      0

/* 保温目标温度偏移: 开机保温时, 目标温度 = brew_temp - 此值
 * 当前设为 0 → 待机温度直接维持在萃取温度 (PID 在 brew_temp ±1℃ 抖动) */
#define WARMUP_TEMP_OFFSET   0.0f

/* 目标压力 (仅用于显示/记录, 无 PID 闭环) */
static int g_target_pressure = TARGET_PRES_DEFAULT;

static volatile uint16_t g_anim_val = 0xFFFF;
static volatile uint8_t g_anim_resp = 0;
static volatile uint8_t g_curve_running = 0;
static volatile uint8_t g_on_curve_page = 0;   /* 当前是否在曲线页(page 4): D218 进入置 1, 离开清 0 */
static uint16_t g_curve_ch0_count = 0;    /* 本次曲线运行期间 CH0 已写点数 (CURVE_DualAppend 调用次数) */
static uint8_t  g_curve_xstep_expanded = 0; /* X 步长切换标志: CH0>=200 后从 4 切到 2 (量程 50s -> 100s), 一次性, 防重复 */
static uint8_t  g_curve_xstep = 4;       /* 当前 X 步长(像素/点): 4=200点可见, 2=400点可见 */
static u32 g_curve_sample_period = 250;   /* CH0 采样周期(ms), 步长2和步长4 都用 250ms 一帧 */

/* ---- 变压模式(曲线页)状态 ----
 * g_curve_drive        : 1=水泵由预设曲线驱动(客户未扭旋钮时, 实际曲线完全跟随预制曲线)
 * g_curve_knob_touched : 1=客户已扭过旋钮, 实际曲线脱离预制曲线, 改由旋钮控制
 * g_preset_edited      : 1=本次会话改动了预设(退出曲线页时可能需要提示保存)
 * g_preset_edit_tick   : 第一次扭旋钮的时刻 (15s 判定起点)
 * g_preset_prompt      : 1="是否保存"弹窗已显示, 等待屏端上报用户选择 */
static uint8_t g_curve_drive = 0;
static uint8_t g_curve_knob_touched = 0;
static uint8_t g_preset_edited = 0;
static u32   g_preset_edit_tick = 0;
static uint8_t g_preset_prompt = 0;
/* 曲线页进入次数 (诊断: 0xD218 被重复上报时会看到 #1 #2 #3 连续刷) */
static uint16_t g_curve_enter_count = 0;

/* 退出曲线页后弹出"是否保存"提示的等待时长:
 * 客户第一次扭旋钮后, 若在此时长内就退出曲线页 -> 视为误操作, 静默丢弃改动;
 * 超过此时长(说明客户确实在编辑) -> 退出时弹窗询问是否保存. */
#define CURVE_EDIT_PROMPT_DELAY_MS   15000u

/* ---- 屏端配合项 ----
 * CURVE_EXIT_VP        : 屏端"退出曲线页"按钮上报的VP (0xD300), 仅在曲线页(page 4)有效.
 *                        只要有上报就算退出, 不看值.
 * CURVE_SAVE_PROMPT_PAGE   : "是否保存"弹窗页号 (page 12)
 * CURVE_SAVE_PROMPT_VP     : 弹窗"是(保存)"按钮上报VP 0xD30C — 上报 1 才有效
 * CURVE_SAVE_PROMPT_VP_CANCEL : 弹窗"否(不保存)"按钮上报VP 0xD308 — 上报 1 才有效
 * CURVE_SAVE_PROMPT_VP_CANCEL2: 另一个"否"按钮VP 0xD304 — 上报非0即视为不保存
 * CURVE_SAVE_PROMPT_BACK   : 选择完成后回哪个页面 (1=主界面)
 *
 * 注意: 任一弹窗按钮VP 未配置(0xFFFF)时, 对应分支不编译;
 *       若"保存"VP 未配置, 固件退化为"静默丢弃改动", 不弹窗也不会误动作. */
#define CURVE_EXIT_VP                  0xD300u
#define CURVE_SAVE_PROMPT_PAGE         12u
#define CURVE_SAVE_PROMPT_VP           0xD30Cu
#define CURVE_SAVE_PROMPT_VP_CANCEL    0xD308u
#define CURVE_SAVE_PROMPT_VP_CANCEL2   0xD304u
#define CURVE_SAVE_PROMPT_BACK         1u

/* 曲线页退出的边沿检测 (屏端上报 CURVE_EXIT_VP 时置位, 主循环消费) */
static volatile uint8_t g_curve_exit_pending = 0;
/* 保存提示弹窗的用户选择 (0=无 1=保存 2=不保存) */
static volatile uint8_t g_preset_choice = 0;
/* 弹窗按钮的上一次上报值 (0→1 边沿判定用, 弹窗显示时复位) */
static uint16_t g_preset_prev_save = 0u;
static uint16_t g_preset_prev_cancel = 0u;

/* 曲线页相关VP处理 (退出按钮 / 保存提示弹窗), 前置声明 */
static void Curve_VPHandle(uint16_t vp_addr, uint16_t value);

/* MCU主动切页: 切到非曲线页时同步清掉"在曲线页"标志.
 *
 * 背景: g_on_curve_page 是"客户在不在曲线页"的缓存标志.
 *   置位: 进曲线页时 (D218)
 *   清零: (a) 屏端上报 0xD300 (客户从曲线页离开)
 *         (b) 本函数 (MCU自己切走的页)
 *   若客户在曲线页用屏端自带的导航直接跳到别的页、而屏端没报 0xD300,
 *   标志就会变粘滞 -> 之后在任何页按KEY1都误走曲线页分支(不跳page3).
 *   所以屏端离开 page4 时必须报 0xD300; 本函数是第二道防线. */
static void Page_SwitchTo(uint16_t page)
{
    if (page != 4u) {
        g_on_curve_page = 0;
    }
    DWIN_SwitchPage(page);
}

/* 预设水泵曲线: 9组 x 400点 档位制, 存放在主Flash 0x0800B000 起 9 个页
 * (见 preset_store.c). 曲线页旋钮 0xD250 的值 0-8 即为槽号. */

static void on_touch(uint8_t icon_id)
{
    printf("Touch: icon=%u\r\n", icon_id);
}

static volatile uint16_t g_d000_val = 0;
static volatile uint8_t  g_d000_resp = 0;

/* 7组选择控件 (SEL1): 触控VP 0xD100起每隔4字节, 共7个, 点击触发翻页 5-11 */
#define SEL_COUNT        7
#define SEL_TOUCH_BASE   0xD100
#define SEL_VP_STEP      4
static volatile int8_t g_sel_click = -1;   /* on_vpread标记被点击的索引, -1=无 */

/* 0xD218 边沿检测: 0 → 1 触发进入曲线绘制页.
 * on_vpread 记 g_d218_pending, 主循环 D218_Curve_Poll 消费:
 *   Beverage_CurveStart() + 写回 0 让屏端回到未选外观. */
#define D218_VP            0xD218u
static volatile uint8_t g_d218_pending = 0;
static uint16_t g_d218_prev = 0;   /* 上次上报值 (边沿判定用), MCU 把 D218 写回 0 时同步清 0 */

/* 灯板三 VP:
 *   0xD04C: 开关状态 (0=灭/1=亮)   ←→ 0xD050 (镜像, 谁变同步另一个)
 *   0xD054: 亮度档位 0-10         → settings.light_level
 * 原 0xD208 / LIGHT_VP_ADDR / g_light_val / g_light_resp / LightBoard_Poll 已删除. */
static volatile uint16_t g_d04c_val = 0;
static volatile uint8_t  g_d04c_resp = 0;
static volatile uint16_t g_d050_val = 0;
static volatile uint8_t  g_d050_resp = 0;
static volatile uint16_t g_d054_val = 0;
static volatile uint8_t  g_d054_resp = 0;

/* 设定类变量地址(迪文屏, 均为u16整数): 开机flash->VP, 运行中VP变化->flash */
#define SET_VP_CUP_COUNT    0xD014   /* 杯数统计 0-9999 */
#define SET_VP_BREW_TEMP    0xD040   /* 目标萃取温度(℃整数) */
#define SET_VP_HEAD_TEMP    0xD044   /* 目标冲泡头温度(℃整数) */
#define SET_VP_BREW_TIME    0xD048   /* 萃取时间(s) */
#define SET_VP_LANG         0xD060   /* 中英文 0=中文 1=英文 */
/* 预浸泡时间: 屏端直接改 u16 (整数秒 1-9), MCU 检测→写入 settings */
#define SET_VP_PREINFUSE_DUR      0xD058  /* 预浸泡持续出水时间(秒, u16, 屏端输入 1-9) */
#define SET_VP_PREINFUSE_STOP     0xD05C  /* 预浸泡后停水时间(秒, u16, 屏端输入 1-9) */
#define SET_VP_PREINFUSE_ENABLE   0xD070  /* 预浸泡开关 (0=跳过预浸泡, 1=执行预浸泡) */

/* 杯数清零触发VP: 0=常态, 1=用户按下重置(0→1边沿触发清零) */
#define CUP_RESET_VP        0xD22C
/* 恢复出厂设置触发VP: 0=常态, 1=用户按下恢复(0→1边沿触发) */
#define RESTORE_VP          0xD080
/* 记录上次的CUP_RESET_VP值, 用于检测 0→1 上升沿 */

static volatile uint16_t g_set_cup_val = 0;
static volatile uint8_t  g_set_cup_resp = 0;
static volatile uint16_t g_set_btemp_val = 0;
static volatile uint8_t  g_set_btemp_resp = 0;
static volatile uint16_t g_set_htemp_val = 0;
static volatile uint8_t  g_set_htemp_resp = 0;
static volatile uint16_t g_set_btime_val = 0;
static volatile uint8_t  g_set_btime_resp = 0;
static volatile uint16_t g_set_lang_val = 0;
static volatile uint8_t  g_set_lang_resp = 0;
static volatile uint16_t g_set_pen_val = 1;     /* 预浸泡开关初值=1(开) */
static volatile uint8_t  g_set_pen_resp = 0;

/* 杯数清零待处理标记: 在on_vpread检测到0xD22C 0→1边沿时置1, 主循环消费 */
static volatile uint8_t  g_cup_reset_pending = 0;
/* 恢复出厂设置待处理标记: 在on_vpread检测到0xD080 0→1边沿时置1, 主循环消费 */
static volatile uint8_t  g_restore_pending = 0;
/* 恢复出厂预设曲线进行中: 1=正在异步写9页, 写完重画CH1 */
static uint8_t  g_preset_restore_active = 0;

/* 屏显温度量化: 0.5℃ 步长四舍五入 (仅用于屏幕显示).
 * 例: 89.2→89.0, 89.3→89.5, 89.4→89.5.
 * 串口 printf 和所有控制逻辑仍用实际温度, 只有写屏前过这个函数. */
static float QuantTemp_ForScreen(float t)
{
    return (float)((int)(t * 2.0f + (t >= 0.0f ? 0.5f : -0.5f))) * 0.5f;
}

/* 刷新屏端 3 个文本控件 (VP 0xEA00/0xEA40/0xEA80): "on时间/off时间" 格式, 整数秒
 * 例: 预浸泡 5s + soak 3s -> "5/3"
 * SP 是描述符 (0x8A00/0x8A40/0x8A80), 文本变量数据 VP 是 0xEA00 系列 (屏端配置)
 *
 * OFF 时强制显示 "0/0": 总开关关闭状态下, 控件不显示具体参数, 仅显示 0/0 标识关闭. */
static void UpdatePreinfuseText(void)
{
    char buf[16];
    const Settings_t *s = Settings_Get();
    if (s->preinfuse_master) {
        /* ON: dur/stop 实参 (整数秒) */
        snprintf(buf, sizeof(buf), "%u/%u",
                 (unsigned)s->preinfuse_duration_s,
                 (unsigned)s->preinfuse_stop_s);
    } else {
        /* OFF: 固定 0/0 */
        buf[0] = '0'; buf[1] = '/'; buf[2] = '0'; buf[3] = '\0';
    }
    DWIN_SetText(0xEA00, buf);
    DWIN_SetText(0xEA40, buf);
    DWIN_SetText(0xEA80, buf);
}

static void on_vpread(uint16_t vp_addr, uint16_t value)
{
    uint8_t k;
#ifdef DEBUG_PRINT_VP_READ
    printf("[vp] read vp=0x%04X val=%u\r\n", vp_addr, value);  /* 调试: 打开此宏看屏端上报 */
#endif
    /* 变量值变化可能触发文字控件颜色变化(由language.c处理,避免重复逻辑) */
    Language_OnTextColorVar(vp_addr, value);
    if (vp_addr == ANIM_VP_ADDR) {
        g_anim_val = value;
        g_anim_resp = 1;
    } else if (vp_addr == DWIN_VP_ADDR_PREINFUSION) {   /* 0xD000 预浸泡 */
        g_d000_val = value;
        g_d000_resp = 1;
    } else if (vp_addr >= SEL_TOUCH_BASE &&
               vp_addr <= SEL_TOUCH_BASE + (SEL_COUNT - 1) * SEL_VP_STEP &&
               ((vp_addr - SEL_TOUCH_BASE) % SEL_VP_STEP) == 0) {
        /* SEL1: 命中某个触控VP, 记录索引, 上报地址即代表点击哪个键(不看值) */
        g_sel_click = (int8_t)((vp_addr - SEL_TOUCH_BASE) / SEL_VP_STEP);
    } else if (vp_addr == D218_VP) {
        /* 0xD218: 0→1 边沿触发进入曲线页.
         * 必须做边沿判定: 控件值若停在1, 屏端任何一次重发都会重新进页,
         * 而 Beverage_CurveStart() 会把 g_curve_drive 清0 -> 变压模式被关掉. */
        if (g_d218_prev == 0u && value == 1u) {
            g_d218_pending = 1;                 /* 主循环 D218_Curve_Poll 消费 */
        }
        g_d218_prev = value;
    } else if (vp_addr == CURVE_EXIT_VP ||
               vp_addr == CURVE_SAVE_PROMPT_VP ||
               vp_addr == CURVE_SAVE_PROMPT_VP_CANCEL ||
               vp_addr == CURVE_SAVE_PROMPT_VP_CANCEL2) {
        /* 曲线页退出按钮 / 保存提示弹窗的"是·否"按钮 (屏端配合项) */
        Curve_VPHandle(vp_addr, value);
    } else if (vp_addr == 0xD04C) {
        g_d04c_val = value;
        g_d04c_resp = 1;
    } else if (vp_addr == 0xD050) {
        g_d050_val = value;
        g_d050_resp = 1;
    } else if (vp_addr == 0xD054) {
        g_d054_val = value;
        g_d054_resp = 1;
    } else if (vp_addr == SET_VP_CUP_COUNT) {   /* 0xD014 杯数 */
        g_set_cup_val = value;
        g_set_cup_resp = 1;
    } else if (vp_addr == SET_VP_BREW_TEMP) {   /* 0xD040 目标萃取温度 */
        g_set_btemp_val = value;
        g_set_btemp_resp = 1;
    } else if (vp_addr == SET_VP_HEAD_TEMP) {   /* 0xD044 目标冲泡头温度 */
        g_set_htemp_val = value;
        g_set_htemp_resp = 1;
    } else if (vp_addr == SET_VP_BREW_TIME) {   /* 0xD048 萃取时间 */
        g_set_btime_val = value;
        g_set_btime_resp = 1;
    } else if (vp_addr == 0xD060) {
        Language_OnD060(value);
        g_set_lang_val = value;                 /* 语言模式同时进设定(掉电保存) */
        g_set_lang_resp = 1;
    } else if (vp_addr == SET_VP_PREINFUSE_DUR) {    /* 0xD058 预浸泡持续 (整数秒 1-9) */
        /* 总开关只是决定是否执行预浸泡, 不干涉 dur/stop 参数持久化 */
        if (value < 1 || value > 9) {
            printf(">>> Set REJECTED: preinfuse dur %u\n", value);
        } else if ((uint16_t)value != Settings_Get()->preinfuse_duration_s) {
            Settings_Edit()->preinfuse_duration_s = (uint16_t)value;
            Settings_MarkDirty();
            printf(">>> Set: preinfuse duration = %u s\n", (unsigned)value);
            UpdatePreinfuseText();
        }
    } else if (vp_addr == SET_VP_PREINFUSE_STOP) {   /* 0xD05C 预浸泡后停水 (整数秒 1-9) */
        if (value < 1 || value > 9) {
            printf(">>> Set REJECTED: preinfuse stop %u\n", value);
        } else if ((uint16_t)value != Settings_Get()->preinfuse_stop_s) {
            Settings_Edit()->preinfuse_stop_s = (uint16_t)value;
            Settings_MarkDirty();
            printf(">>> Set: preinfuse stop = %u s\n", (unsigned)value);
            UpdatePreinfuseText();
        }
    } else if (vp_addr == SET_VP_PREINFUSE_ENABLE) {  /* 0xD070 预浸泡开关 (0/1) */
        g_set_pen_val = value ? 1 : 0;
        g_set_pen_resp = 1;
    } else if (vp_addr == CUP_RESET_VP) {       /* 0xD22C 杯数清零触发: 0→1 边沿 */
        static uint16_t prev_d22c = 0;
        if (prev_d22c == 0u && value == 1u) {
            g_cup_reset_pending = 1;           /* 主循环消费, 避免中断上下文写 flash */
        }
        prev_d22c = value;
    } else if (vp_addr == RESTORE_VP) {         /* 0xD080 恢复出厂设置: 0→1 边沿 */
        static uint16_t prev_d080 = 0;
        if (prev_d080 == 0u && value == 1u) {
            if (g_preset_restore_active) {
                /* 恢复出厂预设正在进行中: 忽略重复触发.
                 * 否则屏端重发 0xD080=1 会反复重启整个恢复流程(每轮40ms关中断),
                 * 表现为"死机". */
                printf(">>> Restore ignored: preset restore already running\r\n");
            } else {
                g_restore_pending = 1;         /* 主循环消费 */
            }
        }
        prev_d080 = value;
    } else {
        /* 语言模块正在读取SP+6以缓存文本数据VP, 把响应转发过去 */
        uint16_t pending = Language_GetPendingReadVP();
        if (pending != 0xFFFF && vp_addr == pending) {
            printf("[vp] lang pending=0x%04X match=1\r\n", pending);  /* [DEBUG] */
            Language_OnVPReadResp(value);
        } else {
            printf("[vp] lang pending=0x%04X match=0 vp=0x%04X\r\n", pending, vp_addr);  /* [DEBUG] */
        }
    }
}

/*==============================================================
 * 0xD000增量控件(数据自动上传) → 触发锅炉完整流程
 * 控件"数据自动上传"开启后, 点击时屏主动发0x83帧 → on_vpread置g_d000_resp.
 * 收到即视为点击, 触发流程. 无需MCU轮询读.
 *==============================================================*/
static void CooldownTest_Init(uint16_t baseline)
{
    g_d000_resp = 0;
    (void)baseline;
}

/*==============================================================
 * 出饮控制: 萃取 / 热水 / 蒸汽 三个流程, 按键触发, 非阻塞状态机
 *   萃取: KEY1(PA5) -> 萃取阀 PB2 + 泵. 开启预浸泡时先低压预浸泡(泵25% = 12.5Hz),
 *         停水 soak, 再全压出水(泵50Hz), 全压时长 brew_time_s.
 *         曲线页(page 4)上 KEY1 改为变压萃取 (Beverage_CurveStartCH0).
 *   热水: KEY2(PA7) -> 热水阀 PB11 + 泵 50Hz, 时长 hot_time_s.
 *   蒸汽: KEY3(PB7) -> 蒸汽阀 PB13 + 蒸汽泵 PB12 + 蒸汽加热器 PA11 同时工作, 最长 BEV_STEAM_RUN_MS.
 *   任一流程出水期间按下任意键 -> 停止; 水箱缺水立即停止. 锅炉流程占用时忽略触发.
 *==============================================================*/
#define BEV_HOT_DUR_MS()   ((u32)Settings_Get()->hot_time_s * 1000u)   /* 热水时长(设定) */
#define BEV_BREW_DUR_MS()  ((u32)Settings_Get()->brew_time_s * 1000u)  /* 萃取时长(设定) */
#define BEV_PREINFUSE_DURATION_MS()  ((u32)Settings_Get()->preinfuse_duration_s * 1000u)  /* 预浸泡持续出水时长(设定, 秒) */
#define BEV_PREINFUSE_STOP_MS()      ((u32)Settings_Get()->preinfuse_stop_s     * 1000u)  /* 预浸泡后 soak 停水时长(设定, 秒) */
#define BEV_PREINFUSE_PUMP_POWER   250     /* 预浸泡泵功率: 25.0% = 12.5Hz (低压低流量) */
#define BEV_STEAM_RUN_MS      180000u /* 蒸汽最长运行时长(ms, 默认3分钟), 到时自动停 */
#define BEV_HOT_PUMP_FREQ     200    /* 热水出水: 泵全速 (0.25Hz单位, 200=50Hz) */
#define BEV_BREW_START_POWER  300    /* 萃取出水起始泵功率(0.1%单位=30.0%), 当前未使用 */
#define BEV_CUP_COUNT_THRESHOLD_MS 15000u  /* 杯数统计阈值: 全压出水满此时长记一杯(ms) */
#define BEV_COLD_DURATION_MS     30000u  /* 降温冲洗(/cold): 泵全功率打水时长(ms), 加热全程关闭 */

typedef enum {
    BEV_IDLE = 0,      /* 空闲 */
    BEV_HOT_RUN,       /* 热水出水进行中 */
    BEV_BREW_HEAT,     /* 萃取: 加热等待到温 (已停用: 萃取出水不再等温, 保留供未来恢复) */
    BEV_BREW_PREINFUSE,/* 萃取: 低压预浸泡(润湿粉饼+释放CO2) */
    BEV_BREW_PREINFUSE_STOP, /* 萃取: 预浸泡停水 soak (泵停,阀关) */
    BEV_BREW_RUN,      /* 萃取: 全压出水进行中 */
    BEV_STEAM_RUN,     /* 蒸汽: 阀+蒸汽泵+加热器全程同时工作 */
    BEV_COLD_RUN       /* 降温冲洗(/cold): 泵全功率打水, 加热功率强制0 */
} Bev_State_t;

static Bev_State_t g_bev_state = BEV_IDLE;
static u32 g_bev_start_tick = 0;
static u8  g_bev_cup_counted = 0;   /* 本次萃取是否已计入杯数(防止重复计数) */

/* 应用 X 轴步长到两条曲线 + 下方时间标签.
 * 图表宽 800px, 步长 = 每数据点占的横向像素:
 *   步长4 -> 800/4 = 200 点可见 (量程 50s)
 *   步长2 -> 800/2 = 400 点可见 (量程 100s)
 * 0xC000 / 0xC010 是两条曲线控件的 SP 描述符地址, 各 +9 偏移为 u16 VP:
 *   高字节 = 数据源通道 (0=CH0 实际, 1=CH1 预设), 低字节 = 像素步长.
 * 必须在预铺曲线之前生效, 否则屏端先用旧步长画完预铺段, 新点会错位.
 * 这是唯一改步长的地方: 两条曲线每次一起改, 保证步长始终一致. */
static void Curve_ApplyXStep(uint8_t step)
{
    g_curve_xstep = step;
    CURVE_SetXStep(0xC000, step, 0);   /* 曲线1: CH0 实际曲线 */
    CURVE_SetXStep(0xC010, step, 1);   /* 曲线2: CH1 预设曲线 */
    CURVE_SetXLabels(step);            /* X 标签: 步长4->5..50s, 步长2->10..100s */
}

/* 按预设曲线的有效点数选步长: >200 点必须用步长2, 否则只看得见前半段.
 * 实际曲线(CH0)同步同一步长, 保证两条曲线在同一 X 尺度上对比. */
static void Curve_ApplyXStepByPreset(void)
{
    uint16_t n = Preset_Length();
    Curve_ApplyXStep((n > 200u) ? 2u : 4u);
}

/* 跳到曲线页(page 4)并初始化双通道曲线 (CH1 = 预设, CH0 = 实际)
 * 主循环按 g_curve_running 控制 CURVE_DualAppend */
static void Beverage_CurveStart(void)
{
    Page_SwitchTo(4);
    delay_ms(300);
    CURVE_Clear();
    /* 注意: g_curve_running 在这里不置 1. CH0 由用户在曲线页按 KEY1 手动启动 (Beverage_CurveStartCH0).
     * 显式清零 g_curve_running: 确保每次进曲线页旋钮作用域都重置为"调 0xD250"
     * (按 KEY1 后才被 Beverage_CurveStartCH0 改写为"调水泵功率"). */
    CURVE_Pause();
    g_curve_running = 0;
    g_curve_ch0_count = 0;          /* CH0 写点计数清零 */
    g_curve_xstep_expanded = 0;       /* X 步长切换标志清零 */
    g_curve_drive = 0;                /* 未按KEY1前不驱动泵 */
    g_curve_knob_touched = 0;
    g_preset_edited = 0;
    g_preset_prompt = 0;
    g_curve_sample_period = CURVE_SAMPLE_PERIOD_MS;  /* 250ms 一帧 */
    /* CH1 预铺当前选中的预设曲线 (档位 -> Y = 档位 x 12.5, 即 0~50Hz -> 0~500).
     * 必须在设好 X 步长之后做, 否则屏端会按旧步长画完预铺段, 新点会错位. */
    Preset_Load((uint8_t)Settings_Get()->curve_knob_val);
    /* 先按预设的有效点数定步长(>200点用步长2), 再按有效点数画(不含结束标志) */
    Curve_ApplyXStepByPreset();
    CURVE_DrawRefFixed(Preset_Data(), Preset_Length(), PRESET_CURVE_MUL_X100, 1);
    g_curve_enter_count++;
    printf(">>> jump page 4 #%u (Y = pump freq, preset slot %u, %u pts, step %u)\r\n",
           (unsigned)g_curve_enter_count,
           (unsigned)Settings_Get()->curve_knob_val,
           (unsigned)Preset_Length(), (unsigned)g_curve_xstep);
}

/* 只停曲线数据 (g_curve_running / CURVE_Pause), 不切页 */
static void Beverage_CurveDataStop(void)
{
    g_curve_running = 0;
    CURVE_Pause();
}

/* 在曲线页手动启动 (KEY1 按下时):
 *   - 开始采样, 把实际水泵频率往 CH0 写 (CH0 曲线录制)
 *   - 同时启动萃取出水流程 (泵+阀+缺水保护+杯数统计), 留在 page 4
 *   - 预设曲线走到结束标志时萃取结束; 按键/缺水/退出曲线页会提前停止 */

/* 前置声明: 完整实现见下方 Beverage_StartBrewEx() 定义.
 * 这里需要它, 因为本函数实现里要调 Beverage_StartBrewEx(temp, 0). */
static void Beverage_StartBrewEx(float temp, uint8_t switch_page);

static void Beverage_CurveStartCH0(void)
{
    g_curve_running      = 1;
    g_curve_ch0_count    = 0;       /* 重置 CH0 计数 */
    g_curve_xstep_expanded = 0;       /* 重置步长切换标志 */
    g_curve_drive        = 1;       /* 变压模式: 水泵由预设曲线驱动 */
    g_curve_knob_touched = 0;       /* 尚未扭过旋钮 -> 实际曲线跟随预制曲线 */
    g_preset_edited      = 0;       /* 本次会话还没改动预设 */
    g_preset_edit_tick   = 0;
    CURVE_Resume();
    /* 复用正常萃取流程: 开阀/缺水保护/杯数照常生效. 萃取时长不用设定的萃取时间,
     * 由预设曲线长度决定: 采样走到结束标志时结束 (见主循环的曲线采样).
     * switch_page=0: 留在曲线页(page 4), 不跳 page 3. */
    Beverage_StartBrewEx(TEMP_GetLastTemperature(), 0);
    if (g_bev_state == BEV_IDLE) {
        /* 萃取没启动(锅炉忙/缺水): 撤销录制. 否则采样会按预设开泵, 而阀没开、可能还没水 */
        g_curve_running = 0;
        g_curve_drive   = 0;
        CURVE_Pause();
        printf(">>> Curve page: brew not started, recording cancelled\r\n");
        return;
    }
    /* 实时曲线从 X=0 重新画 (不出曲线页的第二次萃取也一样, 否则会接在上一条后面画) */
    CURVE_ClearChannel(CURVE_CH0);
    printf(">>> Curve page: KEY1 -> brew+CH0 (drive=%u knob=%u run=%u bev=%d preset[0]=%u)\r\n",
           (unsigned)g_curve_drive, (unsigned)g_curve_knob_touched,
           (unsigned)g_curve_running, (int)g_bev_state,
           (unsigned)Preset_Data()[0]);
}

/* 变压模式: 把水泵频率设成预设曲线的第 idx 点 (档位 -> 0.25Hz单位).
 * idx 超出曲线长度则不动. 档位0 = 停泵(预设可表达"某段关泵"). */
static void Curve_ApplyPresetPump(uint16_t idx)
{
    uint16_t qhz;

    if (idx >= PRESET_POINT_NUM) return;

    qhz = (uint16_t)((uint16_t)Preset_Data()[idx] * PRESET_GEAR_QHZ);
    AC_Pump_SetFrequency(qhz);
    /* AC_Pump_On() 会重置PWM相位计数器(g_toggle_counter=0),
     * 每 250ms 重复调用会把连续方波打断成 24ms 脉冲 -> 泵不工作.
     * 故只在"停->开"的瞬间调用一次. */
    if (qhz > 0u && !AC_Pump_IsOn()) {
        AC_Pump_On();
    }
}

/* 客户在萃取中第一次扭旋钮: 标记预设已被改动, 记录时刻(15s判定起点).
 * 之后实际曲线不再跟随预制曲线, 改由旋钮控制. */
static void Curve_MarkPresetEdited(u32 now)
{
    if (!g_preset_edited) {
        g_preset_edited    = 1;
        g_preset_edit_tick = now;
        printf(">>> Curve: knob touched, preset edit starts (t=%u ms)\r\n",
               (unsigned)now);
    }
    g_curve_knob_touched = 1;
}

/* 退出曲线页时的预设处理:
 *   改过预设 且 距第一次扭旋钮已超过 CURVE_EDIT_PROMPT_DELAY_MS(15s)
 *     -> 弹"是否保存"提示 (屏端配合), 返回1
 *   其它情况(没改过 / 15s内退出)
 *     -> 静默丢弃改动, 把工作区恢复成Flash里的预设, 返回0 */
static uint8_t Curve_PresetEditOnExit(u32 now)
{
    if (!g_preset_edited) {
        return 0u;
    }

    if ((u32)(now - g_preset_edit_tick) < CURVE_EDIT_PROMPT_DELAY_MS) {
        /* 15秒内退出: 视为误操作, 静默丢弃 */
        printf(">>> Curve: exit within %ums, preset edit discarded\r\n",
               (unsigned)CURVE_EDIT_PROMPT_DELAY_MS);
        (void)Preset_Load((uint8_t)Settings_Get()->curve_knob_val);
        g_preset_edited = 0u;
        return 0u;
    }

#if (CURVE_SAVE_PROMPT_PAGE != 0xFFFFu) && (CURVE_SAVE_PROMPT_VP != 0xFFFFu)
    /* 超过15秒: 说明客户确实在编辑 -> 弹窗询问是否保存 */
    g_preset_prompt = 1u;
    g_preset_choice = 0u;
    /* 复位按钮边沿检测 + 把按钮VP清0, 保证弹窗里的第一次点击就是有效边沿 */
    g_preset_prev_save   = 0u;
    g_preset_prev_cancel = 0u;
#if (CURVE_SAVE_PROMPT_VP != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP, 0u);
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP_CANCEL, 0u);
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL2 != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP_CANCEL2, 0u);
#endif
    Page_SwitchTo(CURVE_SAVE_PROMPT_PAGE);
    delay_ms(300);
    printf(">>> Curve: exit after %ums, ask to save preset (slot %u)\r\n",
           (unsigned)CURVE_EDIT_PROMPT_DELAY_MS,
           (unsigned)Settings_Get()->curve_knob_val);
    return 1u;
#else
    /* 屏端弹窗未配置: 退化为静默丢弃 */
    g_preset_prompt = 0u;
    printf(">>> Curve: save prompt not configured, preset edit discarded\r\n");
    (void)Preset_Load((uint8_t)Settings_Get()->curve_knob_val);
    g_preset_edited = 0u;
    return 0u;
#endif
}

/* 保存提示弹窗的用户选择处理 (屏端上报 CURVE_SAVE_PROMPT_VP 后调用).
 * choice: 1=保存 2=取消 */
static void Curve_SaveChoiceHandle(uint8_t choice)
{
    uint8_t slot = (uint8_t)Settings_Get()->curve_knob_val;

    g_preset_prompt = 0u;

    if (choice == 1u) {
        /* 实际曲线和预设长度可能不同(提前停止则更短, 扭旋钮后录满 99s 则可能更长):
         * 结束标志移到实际曲线末尾, 保存的就是实际录到的那一段 */
        if (g_curve_ch0_count > 0u) {
            Preset_SetLength(g_curve_ch0_count);
        }
        if (Preset_Save(slot) == 0u) {
            printf(">>> Curve: preset slot %u saved to flash (%u pts)\r\n",
                   (unsigned)slot, (unsigned)Preset_Length());
        } else {
            printf(">>> Curve: preset slot %u SAVE FAILED\r\n", (unsigned)slot);
            /* 保存失败: 恢复成Flash里的原值, 避免内存里留着没存下的改动 */
            (void)Preset_Load(slot);
        }
    } else {
        /* 取消: 丢弃改动, 恢复Flash里的原值 */
        (void)Preset_Load(slot);
        printf(">>> Curve: preset edit cancelled (slot %u)\r\n", (unsigned)slot);
    }

    g_preset_edited = 0u;
    /* 把弹窗按钮VP写回0, 防止屏端保留高电平导致下次弹窗误判为"已选择".
     * VP 未配置时不发这些帧(0xFFFF不是合法VP). */
#if (CURVE_SAVE_PROMPT_VP != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP, 0u);
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP_CANCEL, 0u);
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL2 != 0xFFFFu)
    DWIN_WriteVar(CURVE_SAVE_PROMPT_VP_CANCEL2, 0u);
#endif
    Page_SwitchTo(CURVE_SAVE_PROMPT_BACK);
    delay_ms(300);
}

/* 曲线页相关的VP上报处理:
 *   CURVE_EXIT_VP        : 屏端"退出曲线页"信号(0xD300) — 只要有上报就算退出, 不看值;
 *                          仅在曲线页(page 4)有效.
 *   CURVE_SAVE_PROMPT_VP : 弹窗"是"按钮(0xD30C)  — 上报 1 才有效 -> 保存
 *   CURVE_SAVE_PROMPT_VP_CANCEL : 弹窗"否"按钮(0xD308) — 上报 1 才有效 -> 不保存
 *   CURVE_SAVE_PROMPT_VP_CANCEL2: 另一个"否"按钮(0xD304) — 上报非0 -> 不保存
 *   弹窗按钮只在弹窗显示期间接受, 避免处理后屏端重复上报导致二次触发 */
static void Curve_VPHandle(uint16_t vp_addr, uint16_t value)
{
    /* 弹窗按钮: 只在弹窗显示期间接受, 避免处理后屏端重复上报导致二次触发 */
    if (g_preset_prompt) {
#if (CURVE_SAVE_PROMPT_VP != 0xFFFFu)
        if (vp_addr == CURVE_SAVE_PROMPT_VP) {
            if (g_preset_prev_save == 0u && value == 1u) {
                g_preset_choice = 1u;      /* 1=保存 */
            }
            g_preset_prev_save = value;
            return;
        }
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL != 0xFFFFu)
        if (vp_addr == CURVE_SAVE_PROMPT_VP_CANCEL) {
            if (g_preset_prev_cancel == 0u && value == 1u) {
                g_preset_choice = 2u;      /* 2=不保存 */
            }
            g_preset_prev_cancel = value;
            return;
        }
#endif
#if (CURVE_SAVE_PROMPT_VP_CANCEL2 != 0xFFFFu)
        if (vp_addr == CURVE_SAVE_PROMPT_VP_CANCEL2) {
            if (value != 0u) {
                g_preset_choice = 2u;      /* 2=不保存 */
            }
            return;
        }
#endif
    }
#if (CURVE_EXIT_VP != 0xFFFFu)
    if (vp_addr == CURVE_EXIT_VP) {
        /* 0xD300: 只要有上报就表示客户退出了曲线页, 不看上报的值.
         * 仅在曲线页有效(其它页面的同名上报一律忽略). */
        if (g_on_curve_page) {
            g_curve_exit_pending = 1u;     /* 主循环消费 */
        }
        return;
    }
#endif
    (void)vp_addr;
    (void)value;
}

/* 退出曲线页回到主界面 (page 1) */
static void Beverage_GoHome(void)
{
    Page_SwitchTo(1);
    delay_ms(300);
    g_on_curve_page = 0;       /* 离开曲线页: KEY1 恢复为正常萃取流程 */
    printf(">>> back to page 1 (main)\r\n");
}

/* 停止当前流程: 停各泵+关各阀+取消加热+恢复锅炉状态机
 * 注: 跳页只在 Beverage_GoHome 调用点显式做, 不在此统一处理 */
static void Beverage_Stop(void)
{
    Bev_State_t prev_state = g_bev_state;

    AC_Pump_Off();
    Boiler_AllValvesClose();
    Boiler_HeatStop();
    Boiler_SetExternalControl(0);
    Steam_PumpOff();            /* 蒸汽泵停 */
    STEAM_HeaterOff();          /* 蒸汽加热停 */
    Steam_ValveClose();         /* 蒸汽阀关 */
    g_bev_state = BEV_IDLE;

    /* 萃取出水计时清零 (BEV_BREW_RUN 结束后下次显示为 0) */
    DWIN_WriteVar(DWIN_VP_ADDR_BREW_ELAPSED, 0);

    /* 任何出饮流程结束都停曲线数据 (避免后台继续 append) */
    Beverage_CurveDataStop();

    /* 萃取流程结束后跳回 page 1; 但变压页(page 4)里启动的萃取留在当前页
     * (热水/蒸汽流程本来就不跳) */
    if (!g_on_curve_page &&
        (prev_state == BEV_BREW_HEAT ||
         prev_state == BEV_BREW_PREINFUSE ||
         prev_state == BEV_BREW_PREINFUSE_STOP ||
         prev_state == BEV_BREW_RUN)) {
        Beverage_GoHome();
    }

    /* 出饮结束后重启保温: 设回 warm_target, 进 HEAT_HOLD.
     * 冲泡头加热片全程运行 (开机已启动), 此处不关闭. */
    {
        float warm_target = Settings_Get()->brew_temp - WARMUP_TEMP_OFFSET;
        Boiler_SetTarget(warm_target);
        Boiler_HeatStart();
    }
}

/* 出饮入口检查: 锅炉状态允许出饮吗?
 * IDLE: 允许
 * HEAT_HOLD: 允许 (保温态, 不动泵/阀, 出饮可强制设目标温度为 brew_temp 后开阀/启泵)
 * 其他 (COOLING/HEATING/HOLDING): 拒绝 — 锅炉正在动泵/阀, 与出饮冲突 */
static uint8_t Boiler_IsAvailableForBeverage(void)
{
    Boiler_State_t s = Boiler_GetState();
    return (s == BOILER_IDLE || s == BOILER_HEAT_HOLD) ? 1 : 0;
}

/* 触发热水流程 (按键2/PA7)
 * 与萃取类似: 锅炉进入 BOILER_HEAT_HOLD 做 PID 加热至 g_target_temp
 * (= settings.brew_temp, 不会超过萃取温度), 同时开热水阀+50Hz 泵出水.
 * HEAT_HOLD 状态本身只动加热器, 不碰阀/泵, 所以外部开的阀/启的泵不受影响
 * (因此不能设 Boiler_SetExternalControl(1), 否则 PID 加热会被跳过). */
static void Beverage_StartHot(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> Hot water: beverage task busy\r\n");
        return;
    }
    if (!Boiler_IsAvailableForBeverage()) {
        printf(">>> Hot water: boiler busy (state=%d)\r\n", (int)Boiler_GetState());
        return;
    }
    if (!WATER_IsPresent()) {
        printf(">>> Hot water: tank empty, aborted\r\n");
        return;
    }

    /* 热水流程: 保温维持 (HEAT_HOLD 状态继续, target 保持 warm_target).
     * 冲泡头加热片独立控温 (开机已启用), 与热水流程无关.
     *
     * 锅炉必须处于 HEAT_HOLD: 若在 IDLE (例如 0xD000 锅炉流程结束后),
     * Boiler_Update 的 IDLE 分支每 10ms 关泵关阀, 热水刚启动就被切断.
     * 已在保温中则不重复 HeatStart (它会清 PID 积分, 出水时水温会掉).
     * HeatStart 会关阀停泵, 所以必须在开阀/启泵之前调用. */
    if (Boiler_GetState() != BOILER_HEAT_HOLD) {
        Boiler_SetTarget(Settings_Get()->brew_temp - WARMUP_TEMP_OFFSET);
        Boiler_HeatStart();
        printf(">>> Hot water: boiler was idle, warm-hold restarted\r\n");
    }
    Boiler_HotValveOpen();                      /* PB11 开 (互锁 PB2 关) */
    AC_Pump_SetFrequency(BEV_HOT_PUMP_FREQ);   /* 50Hz 全速 */
    AC_Pump_On();
    g_bev_start_tick = GetTick();
    g_bev_state = BEV_HOT_RUN;
    printf(">>> Hot water dispensing %us (warmup held, target=%.0fC, any key to stop)\r\n",
           (unsigned)(BEV_HOT_DUR_MS() / 1000u),
           Settings_Get()->brew_temp - WARMUP_TEMP_OFFSET);
}

/* 进入萃取预浸泡段: 开萃取阀, 低压低流量泵启动润湿咖啡粉+释放CO2 */
static void Beverage_BrewPreinfuseStart(u32 now)
{
    Boiler_HeatStart();          /* 关键: 切到 HEAT_HOLD, 否则 Boiler_Update 的 IDLE 分支会把泵/阀关了 */
    /* 萃取加热片已在 Beverage_StartBrewEx 里 HeatStart 启用, 这里不重复启用
     * (重复会清掉 PID 积分). 只刷新目标温度, 保证预浸泡→全压控温连续. */
    BrewHeater_SetTarget(Settings_Get()->head_temp);
    Boiler_ValveOpen();         /* 互锁: 自动关热水阀 */
    AC_Pump_SetPower(BEV_PREINFUSE_PUMP_POWER);   /* 25.0% ≈ 12.5Hz */
    AC_Pump_On();
    g_bev_start_tick = now;
    g_bev_state = BEV_BREW_PREINFUSE;
    printf(">>> Brew: pre-infusion starting (%ums @ %u%% pump, target %u Bar)\r\n",
           (unsigned)BEV_PREINFUSE_DURATION_MS(),
           (unsigned)BEV_PREINFUSE_PUMP_POWER,
           (unsigned)g_target_pressure);
}

/* 进入萃取出水段: 开萃取阀, 泵满功率启动 */
static void Beverage_BrewRunStart(u32 now)
{
    Boiler_HeatStart();          /* 关键: 切到 HEAT_HOLD, 保持出水期间温度 */
    /* 萃取加热片保持启用 (预浸泡阶段已启动), 这里只刷新目标温度,
     * 不重置 PID — 保证预浸泡→全压的控温连续. */
    BrewHeater_SetTarget(Settings_Get()->head_temp);
    Boiler_ValveOpen();         /* 互锁: 自动关热水阀 */
    /* 全压萃取: 泵满功率 50Hz (无压力闭环, 变压模式下可用旋钮手动调) */
    AC_Pump_SetPower(AC_PUMP_POWER_MAX);
    AC_Pump_On();
    g_bev_start_tick = now;
    g_bev_cup_counted = 0;      /* 新一轮萃取: 重置杯数累计标志 */
    g_bev_state = BEV_BREW_RUN;
    printf(">>> Brew: full-pressure dispensing %us @ %d Bar (any key to stop)\r\n",
           (unsigned)(BEV_BREW_DUR_MS() / 1000u), g_target_pressure);
}

/* 触发萃取流程 (按键1/PA5). switch_page=1 跳 page 3, =0 留在当前页 (曲线页里用). */
static void Beverage_StartBrewEx(float temp, uint8_t switch_page)
{
    (void)temp;   /* 萃取出水不再等温, temp 参数保留供未来恢复 */

    if (g_bev_state != BEV_IDLE) {
        printf(">>> Brew: beverage task busy\r\n");
        return;
    }
    if (!Boiler_IsAvailableForBeverage()) {
        printf(">>> Brew: boiler busy (state=%d)\r\n", (int)Boiler_GetState());
        return;
    }
    if (!WATER_IsPresent()) {
        printf(">>> Brew: tank empty, aborted\r\n");
        return;
    }

    /* 出饮前把锅炉目标温度切到萃取温度 (PID 持续加热补偿, 但不等待到温).
     * 冲泡头温度由二次加热片 (PB10) 独立闭环, 目标是屏端「冲泡头温度」设定值. */
    Boiler_SetTarget(Settings_Get()->brew_temp);
    BrewHeater_SetTarget(Settings_Get()->head_temp);   /* 冲泡头控温开机已启动, 这里只刷新目标 */

    /* 切页仅在 switch_page=1 时执行 (曲线页 KEY1 启动时留在 page 4) */
    if (switch_page) {
        Page_SwitchTo(3);
        delay_ms(300);
    }

    /* 萃取出水不等温度: 直接进入预浸泡/全压萃取.
     * 变压模式(曲线页): 整条曲线由预设驱动 -> 跳过预浸泡/soak.
     * 否则 soak 阶段泵会停, 实际曲线就无法与预制曲线完全一致. */
    if (g_curve_drive && g_curve_running) {
        /* 变压模式: 泵由预设曲线驱动.
         * 同时要求 g_curve_running(曲线录制会话在进行中), 这样即使 g_curve_drive
         * 因为异常路径残留, 也不会把普通萃取变成变压模式. */
        printf(">>> Brew: curve mode, pump driven by preset curve\r\n");
        Beverage_BrewRunStart(GetTick());
        Curve_ApplyPresetPump(0);      /* 立即把泵设到预设第0点 */
    } else if (Settings_Get()->preinfuse_master) {
        /* 预浸泡开: 低压预浸泡+soak, 再全压萃取 */
        Beverage_BrewPreinfuseStart(GetTick());
    } else {
        /* 预浸泡关: 跳过预浸泡+soak, 直接全压萃取 */
        printf(">>> Brew: pre-infuse disabled, full-pressure start\r\n");
        Beverage_BrewRunStart(GetTick());
    }
}

/* 触发萃取流程 (按键1/PA5): 跳 page 3 的常规入口. 曲线页 KEY1 用 Beverage_StartBrewEx(0). */
static void Beverage_StartBrew(float temp)
{
    Beverage_StartBrewEx(temp, 1);
}

/* 触发蒸汽流程 (按键3/PB7): 蒸汽阀 + 蒸汽泵 + 蒸汽加热器 全程同时工作
 * (不再分 FILL/HEAT 两段: 加热与进水同时进行) */
static void Beverage_StartSteam(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> Steam: beverage task busy\r\n");
        return;
    }
    if (!Boiler_IsAvailableForBeverage()) {
        printf(">>> Steam: boiler busy (state=%d)\r\n", (int)Boiler_GetState());
        return;
    }
    if (!WATER_IsPresent()) {
        printf(">>> Steam: tank empty, aborted\r\n");
        return;
    }

    /* 蒸汽流程: 不动锅炉 target / 状态, 维持保温 (warm_target).
     * 蒸汽加热由独立蒸汽加热器 (PA11/TIM1_CH4) 完成, 不需要锅炉 PID 控温. */
    /* 全程同时: 阀开 + 蒸汽泵抽 + 加热器满功率 */
    Steam_ValveOpen();      /* PB13 开 (蒸汽输出) */
    Steam_PumpOn();         /* PB12 高 (进水) */
    STEAM_HeaterOn();       /* PA11/TIM1_CH4 满功率加热 */
    g_bev_start_tick = GetTick();
    g_bev_state = BEV_STEAM_RUN;
    printf(">>> Steam running (pump + heater on), max %us (any key to stop)\r\n",
           (unsigned)(BEV_STEAM_RUN_MS / 1000u));
}

/*==============================================================
 * 串口命令处理 (供 cmd_console.c 调用, 与面板按键等价)
 *==============================================================*/
#if SYS_UART_CMD_ENABLE
void Cmd_Trigger_Extract(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> CMD /extract: beverage task busy\r\n");
        return;
    }
    DWIN_BuzzerPlayOnce();
    Beverage_StartBrew(TEMP_GetLastTemperature());
}

/* 变压模式(曲线页)萃取: 跳 page 4 + 预铺 CH1 预制曲线 + 按预设曲线驱动水泵 + 录 CH0.
 * 等价于"在曲线页按 KEY1", 只是从串口一步到位(进页 + 启动), 便于调试.
 * 与 /extract 的区别: /extract 是普通萃取(预浸泡+满压, 会跳 page 3),
 * /curve 留在 page 4 且水泵跟随预设曲线. */
void Cmd_Trigger_Curve(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> CMD /curve: beverage task busy\r\n");
        return;
    }
    if (!WATER_IsPresent()) {
        printf(">>> CMD /curve: tank empty, aborted\r\n");
        return;
    }
    DWIN_BuzzerPlayOnce();
    Beverage_CurveStart();      /* 跳 page 4 + 预铺 CH1 预制曲线 */
    g_on_curve_page = 1;
    Beverage_CurveStartCH0();   /* 启动变压萃取(留在 page 4, 泵跟预设) */
}

void Cmd_Trigger_HotWater(void){
    if (g_bev_state != BEV_IDLE) {
        printf(">>> CMD /hot: beverage task busy\r\n");
        return;
    }
    DWIN_BuzzerPlayOnce();
    Beverage_StartHot();
}

void Cmd_Trigger_Steam(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> CMD /steam: beverage task busy\r\n");
        return;
    }
    DWIN_BuzzerPlayOnce();
    Beverage_StartSteam();
}

/* 降温冲洗: 泵全功率打水30秒, 期间锅炉加热功率强制 0.
 * Boiler_HeatStop() 让锅炉回 IDLE → Boiler_Update 的 IDLE 分支会 HEATER_Off(),
 * 确保 PID 不会把加热功率写回来. */
void Cmd_Trigger_Cold(void)
{
    if (g_bev_state != BEV_IDLE) {
        printf(">>> CMD /cold: beverage task busy\r\n");
        return;
    }
    if (!WATER_IsPresent()) {
        printf(">>> CMD /cold: tank empty, aborted\r\n");
        return;
    }

    DWIN_BuzzerPlayOnce();
    /* 关键: 锅炉外部接管. Boiler_Update() 在 external_ctrl=1 时直接 return,
     * 不会进 IDLE 分支调 AC_Pump_Off() 把泵关掉, 也不会动加热器.
     * 泵/阀/加热全部由本流程 (Beverage_Update 的 BEV_COLD_RUN) 独占控制. */
    Boiler_SetExternalControl(1);
    Boiler_HeatStop();                 /* 锅炉回 IDLE (显示为待机) */
    Boiler_HotValveOpen();             /* 打开水阀 PB11 (互锁: 自动关萃取阀 PB2) */
    AC_Pump_SetPower(AC_PUMP_POWER_MAX); /* 100.0% = 50Hz 满功率 */
    AC_Pump_On();
    g_bev_start_tick = GetTick();
    g_bev_state = BEV_COLD_RUN;
    printf(">>> CMD /cold: valve PB11 open, pump 100%% for %us, heater OFF\r\n",
           (unsigned)(BEV_COLD_DURATION_MS / 1000u));
}
#endif /* SYS_UART_CMD_ENABLE */

/* 主循环快速节拍调用: 流程推进/超时/缺水保护 */
static void Beverage_Update(u32 now, float temp)
{
    switch (g_bev_state) {
    case BEV_HOT_RUN:
        if (now - g_bev_start_tick >= BEV_HOT_DUR_MS()) {
            printf(">>> Hot water done\r\n");
            Beverage_Stop();
        } else if (!WATER_IsPresent()) {
            printf(">>> Hot water: tank ran dry, stopped\r\n");
            Beverage_Stop();
        }
        break;

    case BEV_BREW_HEAT:
        if (temp >= Boiler_GetTarget() - BOILER_TEMP_TOL) {
            /* 到温: 预浸泡开 → 预浸泡+soak; 关 → 直接全压萃取 */
            if (Settings_Get()->preinfuse_master) {
                Beverage_BrewPreinfuseStart(now);
            } else {
                printf(">>> Brew: pre-infuse disabled, full-pressure start\r\n");
                Beverage_BrewRunStart(now);
            }
        }
        break;

    case BEV_BREW_PREINFUSE:
        /* 预浸泡: 低压低流量保持 BEV_PREINFUSE_PUMP_POWER 恒定 */
        if (now - g_bev_start_tick >= BEV_PREINFUSE_DURATION_MS()) {
            /* 预浸泡结束: 关泵关阀, 进入停水 soak 阶段让咖啡粉吸水排气 */
            AC_Pump_Off();
            Boiler_ValveClose();
            g_bev_start_tick = now;
            g_bev_state = BEV_BREW_PREINFUSE_STOP;
            printf(">>> Brew: pre-infusion done, soak %u ms before full pressure\r\n",
                   (unsigned)BEV_PREINFUSE_STOP_MS());
        } else if (!WATER_IsPresent()) {
            printf(">>> Brew: tank ran dry during pre-infusion, stopped\r\n");
            Beverage_Stop();
        }
        break;

    case BEV_BREW_PREINFUSE_STOP:
        /* 预浸泡 soak: 泵停阀关, 等咖啡粉吸水稳定 */
        if (now - g_bev_start_tick >= BEV_PREINFUSE_STOP_MS()) {
            /* soak 结束: 进入全压萃取 */
            printf(">>> Brew: soak done, switching to full pressure\r\n");
            Beverage_BrewRunStart(now);
        } else if (!WATER_IsPresent()) {
            printf(">>> Brew: tank ran dry during pre-infuse soak, stopped\r\n");
            Beverage_Stop();
        }
        break;

    case BEV_BREW_RUN:
        /* 杯数统计: 全压出水满15秒视为一杯 (不论后续是否完成/缺水/按键停止) */
        if (!g_bev_cup_counted && (now - g_bev_start_tick) >= BEV_CUP_COUNT_THRESHOLD_MS) {
            g_bev_cup_counted = 1;
            if (Settings_Get()->cup_count < SETTINGS_CUP_COUNT_MAX) {
                Settings_Edit()->cup_count++;
                Settings_MarkDirty();
                DWIN_WriteVar(SET_VP_CUP_COUNT, Settings_Get()->cup_count);
                printf(">>> Cup counted! Total = %u\r\n", Settings_Get()->cup_count);
            } else {
                printf(">>> Cup counted! (max=%u reached)\r\n", (unsigned)SETTINGS_CUP_COUNT_MAX);
            }
        }
        /* 变压萃取(曲线页)不按设定的萃取时间停: 时长由预设曲线长度决定,
         * 曲线采样走到结束标志时结束 (见主循环的曲线采样) */
        if (!(g_curve_drive && g_curve_running) &&
            now - g_bev_start_tick >= BEV_BREW_DUR_MS()) {
            printf(">>> Brew done\r\n");
            Beverage_Stop();
        } else if (!WATER_IsPresent()) {
            printf(">>> Brew: tank ran dry, stopped\r\n");
            Beverage_Stop();
        }
        break;

    case BEV_STEAM_RUN:
        /* 蒸汽泵+加热器 全程持续, 直到超时/缺水/按键 */
        if (now - g_bev_start_tick >= BEV_STEAM_RUN_MS) {
            printf(">>> Steam: timeout, stopped\r\n");
            Beverage_Stop();
        } else if (!WATER_IsPresent()) {
            printf(">>> Steam: tank ran dry, stopped\r\n");
            Beverage_Stop();
        }
        break;

    case BEV_COLD_RUN:
        /* 降温冲洗: 泵全功率打水, 加热器保持关闭(锅炉已在 IDLE).
         * 每周期再压一次 HEATER_Off(), 防外部状态把加热功率写回来. */
        HEATER_Off();
        if (now - g_bev_start_tick >= BEV_COLD_DURATION_MS) {
            printf(">>> Cold: %us elapsed, stopped\r\n",
                   (unsigned)(BEV_COLD_DURATION_MS / 1000u));
            Beverage_Stop();
        } else if (!WATER_IsPresent()) {
            printf(">>> Cold: tank ran dry, stopped\r\n");
            Beverage_Stop();
        }
        break;

    default:
        break;
    }
}

/* 主循环调用: 收到0xD000自动上传即启动锅炉流程 */
static void CooldownTest_Poll(u32 now)
{
    (void)now;
    if (g_d000_resp) {
        g_d000_resp = 0;
        if (g_bev_state != BEV_IDLE) {
            printf(">>> D000 click ignored: beverage task running\r\n");
            return;
        }
        printf(">>> D000 click (val=%u) start boiler cycle\r\n", g_d000_val);
        Boiler_StartCycle();
    }
}

/*==============================================================
 * SEL1 (0xD100-0xD118 触控VP): 点击 → 跳页 5-11
 *==============================================================*/
static void SelPoll_Init(void)
{
    g_sel_click = -1;
}

static void SelPoll_Run(u32 now)
{
    (void)now;
    if (g_sel_click >= 0) {
        int8_t sel = g_sel_click;
        g_sel_click = -1;

        printf(">>> SEL touch[%d] (VP=0x%04X)\r\n",
               sel, (unsigned)(SEL_TOUCH_BASE + sel * SEL_VP_STEP));

        if (sel == 0) {
            Page_SwitchTo(5);
            delay_ms(300);
            printf(">>> jump page 5\r\n");
        } else if (sel == 1) {
            Page_SwitchTo(6);
            delay_ms(300);
            printf(">>> jump page 6\r\n");
        } else if (sel == 2) {
            Page_SwitchTo(7);
            delay_ms(300);
            printf(">>> jump page 7\r\n");
        } else if (sel == 3) {
            Page_SwitchTo(8);
            delay_ms(300);
            printf(">>> jump page 8\r\n");
        } else if (sel == 4) {
            Page_SwitchTo(9);
            delay_ms(300);
            printf(">>> jump page 9\r\n");
        } else if (sel == 5) {
            Page_SwitchTo(10);
            delay_ms(300);
            printf(">>> jump page 10\r\n");
        } else if (sel == 6) {
            Page_SwitchTo(11);
            delay_ms(300);
            printf(">>> jump page 11\r\n");
        }
    }
}

/*==============================================================
 * 0xD218 边沿消费: 0→1 触发进入曲线绘制页, 完后把 D218 写回 0.
 *==============================================================*/
static void D218_Curve_Poll(u32 now)
{
    (void)now;
    if (g_d218_pending) {
        g_d218_pending = 0;
        /* 把控件写回 0, 让屏端回到"未选中"外观.
         * 不写回的话控件值一直是1, 屏端重发1会被当成又一次点击 -> 重复进页.
         * 边沿记忆要跟着清 0: 屏端这个控件是 0/1 循环的增量调节, 写回 0 后下一次点击上报 1;
         * 若 g_d218_prev 还停在 1, 这次点击会被当成"值没变"吞掉, 要再点两下(1->0->1)才进页. */
        DWIN_WriteVar(D218_VP, 0);
        g_d218_prev = 0;
        Beverage_CurveStart();
        g_on_curve_page = 1;   /* 进入曲线页: KEY1 按下时仅启动 CH0 录制, 不跳 page 3 */
    }
}

/*==============================================================
 * 灯板 PWM 调光控制 (合并原 LightBoard_Poll + D04C_Lock_Poll):
 *   - 0xD04C / 0xD050: 互为镜像 (都是 0/1 开关)
 *       D04C=0 → PWM duty 0% (全低, 灯灭)
 *       D04C=1 → PWM duty = D054 * 10% (按亮度档位)
 *   - 0xD054: 亮度档 0-10 (settings.light_level)
 *       D04C=0 → 屏端 D054 强制 0, settings.light_level 冻结保留偏好
 *       D04C=1 → 屏端 D054 从 settings.light_level 恢复 (开灯自动还原)
 *       D054 在 D04C=0 期间被屏端拨动 → 强制回 0 (互锁保护)
 *   - D054 变更仅在 D04C=1 时写入 settings
 *   - PWM 输出由 light_pwm 模块 (PB1 = TIM3_CH4, 1kHz) 驱动
 *   - 500ms 主动轮询 0xD04C, 兼容屏端未开自动上传
 *==============================================================*/

/* 根据当前 D04C/D054 状态计算并输出 PWM
 *   D04C=0 → 全低 (duty 0)
 *   D04C=1 → D054 * 10% duty (D054 限幅 0-10) */
static void LightPWM_ApplyFromState(void)
{
    if (g_d04c_val == 0) {
        LightPWM_Off();
    } else {
        uint16_t level = g_d054_val;
        if (level > 10) level = 10;
        LightPWM_SetLevel((uint8_t)level);
    }
}

/* 应用新开关状态: D04C/D050 镜像同步 + D054 互锁 + PWM 输出 */
static void Switch_ApplyState(uint8_t state)
{
    /* 同步 D050 (镜像) */
    if ((uint8_t)g_d050_val != state) {
        g_d050_val = state;
        DWIN_WriteVar(0xD050, state);
    }
    /* 同步 D04C (镜像) */
    if ((uint8_t)g_d04c_val != state) {
        g_d04c_val = state;
        DWIN_WriteVar(0xD04C, state);
    }

    /* 互锁 D054: D04C=0 强制 0; D04C=1 从 settings.light_level 恢复 */
    uint16_t d054_target = state ? Settings_Get()->light_level : 0;
    if (d054_target > 10) d054_target = 10;
    if (g_d054_val != d054_target) {
        g_d054_val = d054_target;
        DWIN_WriteVar(0xD054, d054_target);
    }

    /* 应用 PWM (按最终 D04C/D054 状态计算) */
    LightPWM_ApplyFromState();
}

static void D04C_Light_Poll(u32 now)
{
    static u32 poll_tick = 0;

    /* 周期主动轮询 0xD04C */
    if (now - poll_tick >= 500) {
        poll_tick = now;
        DWIN_ReadVar(0xD04C, 1);
    }

    /* D04C 或 D050 任一上报: 应用新状态 (PWM + 镜像 + 互锁) */
    if (g_d04c_resp) {
        g_d04c_resp = 0;
        Switch_ApplyState((g_d04c_val != 0) ? 1 : 0);
    }
    if (g_d050_resp) {
        g_d050_resp = 0;
        Switch_ApplyState((g_d050_val != 0) ? 1 : 0);
    }

    /* D054 上报:
     *   D04C=0: 强制 D054=0 (互锁保护, 不写 settings)
     *   D04C=1: 写入 settings.light_level */
    if (g_d054_resp) {
        g_d054_resp = 0;
        uint16_t val = g_d054_val;
        if (val > 10) val = 10;

        if (g_d04c_val == 0) {
            /* D04C=0: 强制回 0, settings 冻结 */
            if (val != 0) {
                g_d054_val = 0;
                DWIN_WriteVar(0xD054, 0);
            }
        } else {
            /* D04C=1: 写入 settings */
            if (val != Settings_Get()->light_level) {
                Settings_Edit()->light_level = val;
                Settings_MarkDirty();
                printf(">>> Set: light level = %u\r\n", val);
            }
        }
        /* D054 变化后, 重新计算并输出 PWM (D04C=0 时保持 0%, D04C=1 时按 D054) */
        LightPWM_ApplyFromState();
    }
}

/*==============================================================
 * 设定类VP双向同步 (0xD014/0xD040/0xD044/0xD048/0xD060):
 *   - 开机: flash设定值写入各VP (main初始化段)
 *   - 运行: VP值变化(控件自动上传, 或500ms轮流轮询应答)
 *     -> 范围校验 -> 更新内存镜像 + 立即应用(萃取温度->锅炉目标)
 *     -> MarkDirty延迟落盘flash
 *==============================================================*/
static void SettingsVP_Apply(uint16_t vp, uint16_t val)
{
    Settings_t *s = Settings_Edit();

    switch (vp) {
    case SET_VP_CUP_COUNT:
        if (val == s->cup_count) break;   /* 无变化 */
        if (val <= SETTINGS_CUP_COUNT_MAX) {
            s->cup_count = val;
            Settings_MarkDirty();
            printf(">>> Set: cup count = %u\r\n", val);
        } else {
            printf(">>> Set REJECTED: cup count %u\r\n", val);
        }
        break;
    case SET_VP_BREW_TEMP:
        if ((float)val == s->brew_temp) break;
        if (val >= 20 && val <= 110) {
            s->brew_temp = (float)val;
            /* 保温中 (HEAT_HOLD 且未出饮): 调整保温目标为 (新brew_temp - 7.5)
             * 出饮中: 立即切到新 brew_temp, 让 PID 加热到新目标
             * 其他: 立即生效 */
            if (g_bev_state == BEV_IDLE
                && Boiler_GetState() == BOILER_HEAT_HOLD) {
                Boiler_SetTarget((float)val - WARMUP_TEMP_OFFSET);
            } else {
                Boiler_SetTarget((float)val);
            }
            Settings_MarkDirty();
            printf(">>> Set: brew temp = %u C\r\n", val);
        } else {
            printf(">>> Set REJECTED: brew temp %u\r\n", val);
        }
        break;
    case SET_VP_HEAD_TEMP:
        if ((float)val == s->head_temp) break;
        if (val >= 20 && val <= 110) {
            s->head_temp = (float)val;
            /* 冲泡头加热片正在控温时, 设定温度立即生效 */
            BrewHeater_SetTarget(s->head_temp);
            Settings_MarkDirty();
            printf(">>> Set: head temp = %u C\r\n", val);
        } else {
            printf(">>> Set REJECTED: head temp %u\r\n", val);
        }
        break;
    case SET_VP_BREW_TIME:
        if (val == s->brew_time_s) break;
        if (val >= 15 && val <= 99) {
            s->brew_time_s = val;
            Settings_MarkDirty();
            printf(">>> Set: brew time = %us\r\n", val);
        } else {
            printf(">>> Set REJECTED: brew time %u\r\n", val);
        }
        break;
    case SET_VP_PREINFUSE_ENABLE:
        if (val == s->preinfuse_master) break;
        if (val <= 1) {
            s->preinfuse_master = val ? 1 : 0;
            Settings_MarkDirty();
            printf(">>> Set: preinfuse %s\n", val ? "ON" : "OFF");
            UpdatePreinfuseText();   /* 开关状态变化, 文本"0/0" 或 "dur/stop" 同步刷新 */
        } else {
            printf(">>> Set REJECTED: preinfuse enable %u\n", val);
        }
        break;
    case SET_VP_LANG:
        if (val == s->lang_mode) break;
        if (val <= 1) {
            s->lang_mode = val;
            Settings_MarkDirty();
            printf(">>> Set: lang = %s\r\n", val ? "EN" : "CN");
        } else {
            printf(">>> Set REJECTED: lang %u\r\n", val);
        }
        break;
    default:
        break;
    }
}

static void SettingsVP_Poll(u32 now)
{
    static u32 poll_tick = 0;
    static u8 poll_idx = 0;
    /* 轮询列表: 5 个 u16 VP + 2 个 u16 预浸泡 (屏端只能改 u16, 不轮询 float VP 了) */
    static const struct { uint16_t vp; uint8_t data_len; } vp_list[8] = {
        { SET_VP_CUP_COUNT,            1 },
        { SET_VP_BREW_TEMP,            1 },
        { SET_VP_HEAD_TEMP,            1 },
        { SET_VP_BREW_TIME,            1 },
        { SET_VP_LANG,        1 },
        { SET_VP_PREINFUSE_DUR,        1 },
        { SET_VP_PREINFUSE_STOP,       1 },
        { SET_VP_PREINFUSE_ENABLE,     1 },
    };

    /* 收到上报(自动上传或轮询应答): 校验并更新设定 */
    if (g_set_cup_resp) {
        g_set_cup_resp = 0;
        SettingsVP_Apply(SET_VP_CUP_COUNT, g_set_cup_val);
    }
    if (g_set_btemp_resp) {
        g_set_btemp_resp = 0;
        SettingsVP_Apply(SET_VP_BREW_TEMP, g_set_btemp_val);
    }
    if (g_set_htemp_resp) {
        g_set_htemp_resp = 0;
        SettingsVP_Apply(SET_VP_HEAD_TEMP, g_set_htemp_val);
    }
    if (g_set_btime_resp) {
        g_set_btime_resp = 0;
        SettingsVP_Apply(SET_VP_BREW_TIME, g_set_btime_val);
    }
    if (g_set_lang_resp) {
        g_set_lang_resp = 0;
        SettingsVP_Apply(SET_VP_LANG, g_set_lang_val);
    }
    if (g_set_pen_resp) {
        g_set_pen_resp = 0;
        SettingsVP_Apply(SET_VP_PREINFUSE_ENABLE, g_set_pen_val);
    }

    /* 杯数清零触发: 0xD22C 0→1边沿. 在主循环处理避免中断上下文写flash */
    if (g_cup_reset_pending) {
        g_cup_reset_pending = 0;
        Settings_Edit()->cup_count = 0;
        Settings_MarkDirty();
        DWIN_WriteVar(SET_VP_CUP_COUNT, 0u);
        printf(">>> Cup count reset to 0 (triggered by 0xD22C edge)\r\n");
    }

    /* 恢复出厂设置: 0xD080 0→1边沿. 装默认值+立即落盘+同步硬件+同步屏 */
    if (g_restore_pending) {
        g_restore_pending = 0;
        Settings_LoadDefaults();                 /* RAM 镜像装默认 */
        (void)Settings_Save();                  /* 立即落盘 (0=成功, 失败不影响运行) */
        /* 同步应用到硬件(对齐开机序列 L903-906/L965-974) */
        Boiler_SetTarget(Settings_Get()->brew_temp);
        g_target_pressure = (int)(Settings_Get()->target_pres + 0.5f);
        {
            /* 灯板: light_level 独立档位 (0=熄灭) -> PWM 输出 */
            uint16_t light_level = Settings_Get()->light_level;
            if (light_level > 10) light_level = 10;
            uint8_t switch_on = (light_level > 0) ? 1 : 0;
            g_d04c_val = switch_on;
            g_d050_val = switch_on;
            g_d054_val = light_level;
            LightPWM_ApplyFromState();  /* 应用 PWM 输出 (D04C=0 时自动 0%) */
            /* 屏幕背光: brightness 独立档位 */
            uint16_t backlight = Settings_Get()->brightness;
            if (backlight > 10) backlight = 10;
            DWIN_SetBacklight((u8)(backlight * 10u),
                              (u8)(backlight * 10u), 0u);
            /* 同步到屏(对齐开机序列) */
            DWIN_WriteVar(0xD04C, switch_on);              /* 开关 0/1 */
            DWIN_WriteVar(0xD050, switch_on);              /* 开关镜像 0/1 */
            DWIN_WriteVar(0xD054, light_level);            /* 亮度档 0-10 */
        }
        DWIN_WriteVar(SET_VP_CUP_COUNT, Settings_Get()->cup_count);
        DWIN_WriteVar(SET_VP_BREW_TEMP, (uint16_t)(Settings_Get()->brew_temp + 0.5f));
        DWIN_WriteVar(SET_VP_HEAD_TEMP, (uint16_t)(Settings_Get()->head_temp + 0.5f));
        DWIN_WriteVar(SET_VP_BREW_TIME, Settings_Get()->brew_time_s);
        DWIN_WriteVar(SET_VP_LANG, Settings_Get()->lang_mode);
        DWIN_WriteVar(SET_VP_PREINFUSE_DUR,     Settings_Get()->preinfuse_duration_s);
        DWIN_WriteVar(SET_VP_PREINFUSE_STOP,    Settings_Get()->preinfuse_stop_s);
        DWIN_WriteVar(SET_VP_PREINFUSE_ENABLE,  Settings_Get()->preinfuse_master);
        UpdatePreinfuseText();                          /* 刷新 0x8A00/0x8A40/0x8A80 */
        DWIN_WriteVar(0xD250, Settings_Get()->curve_knob_val);  /* 曲线页旋钮值(flash 持久化) */
        /* 语言模块感知新默认(虽然g_d060可能不变, 主动同步保险) */
        Language_OnD060(Settings_Get()->lang_mode);
        /* 恢复出厂后: 若当前在保温态 (HEAT_HOLD 且未出饮), 用新 brew_temp 重设保温目标 */
        if (g_bev_state == BEV_IDLE
            && Boiler_GetState() == BOILER_HEAT_HOLD) {
            Boiler_SetTarget(Settings_Get()->brew_temp - WARMUP_TEMP_OFFSET);
        }
        /* 冲泡头加热片若正在控温, 同步恢复默认设定温度 */
        BrewHeater_SetTarget(Settings_Get()->head_temp);
        /* 恢复出厂预设曲线: 9组出厂曲线写回Flash.
         * 异步执行(每100ms写一页, 单次关中断~40ms), 避免连续9页卡住主循环360ms. */
        Preset_SetCurrentSlot((uint8_t)Settings_Get()->curve_knob_val);
        if (Preset_RestoreAllStart() == 0u) {
            g_preset_restore_active = 1;
        }
        (void)Preset_Load((uint8_t)Settings_Get()->curve_knob_val);
        /* 把控件写回0, 避免屏端重发 0xD080=1 反复触发恢复出厂 */
        DWIN_WriteVar(RESTORE_VP, 0);
        printf(">>> Factory defaults restored (triggered by 0xD080 edge)\r\n");
    }

    /* 周期轮流读取(每500ms读一个VP, 每约3.5s一轮), 兼容未开自动上传的控件 */
    if (now - poll_tick >= 500) {
        poll_tick = now;
        DWIN_ReadVar(vp_list[poll_idx].vp, vp_list[poll_idx].data_len);
        poll_idx = (u8)((poll_idx + 1) % 8);
    }
}
int main(void)
{
    float temperature;
    float pressure;
    u32 last_pid_tick;
    u32 last_display_tick;
    u32 last_pres_ctrl_tick;
    uint16_t prev_val = 0xFFFF;
    uint8_t stable_cnt = 0;

    delay_init(48);
    uart_init(115200);
    DWIN_UART_Init(DWIN_DEFAULT_BAUDRATE);
    /* 客户设定: Flash末页 -> 内存镜像(空片/无效则写默认值) */
    Settings_Init();
    GPIO_CtrlInitAll();
    /* PB1 灯板: 硬件 PWM 调光 (light_pwm.c, TIM3_CH4/AF1, 1kHz/10级)
     *   - 复用推挽, PB1 -> AF1 = TIM3_CH4
     *   - PSC=47, ARR=999 → 48MHz/(48*1000) = 1kHz
     *   - 占空比 = LightPWM_SetLevel(0~10) * 10%, 与原软件 PWM 档位一致
     * 0xD04C / 0xD050: 0/1 开关 (互为镜像)
     *   =0 -> CCR=0 (灯灭)
     *   =1 -> PB1 输出 1kHz PWM, 占空比 = D054*10%
     * 0xD054: 0-10 亮度档 (settings.light_level) */
    LightPWM_Init();
    {
        uint16_t light_level = Settings_Get()->light_level;
        if (light_level > 10) light_level = 10;
        uint8_t switch_on = (light_level > 0) ? 1 : 0;
        g_d04c_val = switch_on;
        g_d050_val = switch_on;
        g_d054_val = light_level;
        LightPWM_ApplyFromState();  /* 初始 PWM 输出 */
    }
    TEMP_Sensor_Init();
    OTEMP_Sensor_Init();   /* 出水口温度传感器 (PA0/ADC_CH0) */
    WATER_Sensor_Init();   /* 锅炉水位传感器 (PC13, 低=有水) */
    PRES_Sensor_Init();
    HEATER_Init();
    BrewHeater_Init();     /* 冲泡头加热片 (PB10 继电器通断, 默认不加热) */
    STEAM_HeaterInit();   /* 蒸汽加热器 (PA11/TIM1_CH4) */
    AC_Pump_Init();
    CURVE_Init();
    Preset_Init();    /* 扫描 9 组预设的 Flash 槽, 打印有效槽数 */
    Preset_Load((uint8_t)Settings_Get()->curve_knob_val);  /* 载入当前选中的那一组 */
    Encoder_Init();
    Boiler_Init();   /* 锅炉加热+阀门, 上电待机(不加热, 等触发) */
    /* 开机即启动保温: 目标温度 = 萃取温度 (WARMUP_TEMP_OFFSET=0)
     * HEAT_HOLD 状态自动 PID 恒温, 实际温度在 萃取温度 ±1℃ 抖动 */
    {
        float warm_target = Settings_Get()->brew_temp - WARMUP_TEMP_OFFSET;
        Boiler_SetTarget(warm_target);
        Boiler_HeatStart();   /* 进 HEAT_HOLD: PID 加热到 warm_target, 不动泵/阀 */
        printf(">>> Warmup: target=%.1f C (brew_temp=%.1f C)\r\n",
               warm_target, Settings_Get()->brew_temp);
    }
    /* 开机即启动冲泡头控温: 继电器滞回开关把冲泡头维持在
     * [设定−HIGH, 设定] 区间 (例设定85℃ → 81~85℃), 全程不关 */
    BrewHeater_HeatStart(Settings_Get()->head_temp);
    printf(">>> BrewHeat: target=%.0f C (on<%.0f, off>=%.0f)\r\n",
           Settings_Get()->head_temp,
           Settings_Get()->head_temp - BREW_HYST_LOW_C,
           Settings_Get()->head_temp - BREW_HYST_HIGH_C);
    Steam_Init();    /* 蒸汽锅炉继电器(PB12泵/PB13输出), 默认关 */
    KeyMonitor_Init();   /* 3路按键输入监测(关=低/开=高), 消抖+事件 */
    g_target_pressure = (int)(Settings_Get()->target_pres + 0.5f);   /* 目标压力(断电保存) */

    printf("Coffee Machine Controller - STM32F0\r\n");

    /* 让迪文屏从界面0重新开始 */
    Page_SwitchTo(0);
#if 0
    /* 重置开机动画状态变量(0xA000), 避免上次的动画终值影响本次 */
    DWIN_WriteVar(ANIM_VP_ADDR, 1);
    delay_ms(50);
    DWIN_WriteVar(ANIM_VP_ADDR, 0);
#endif
    delay_ms(500);
   
    DWIN_RegisterTouchCallback(on_touch);
    DWIN_RegisterVPReadCallback(on_vpread);
    /* 不再注册 float VP 回调: 预浸泡用整数秒 u16 即可, 屏端直接改 0xD058/0xD05C */
    DWIN_SetTouchVP(BTN_ICON_ID, BTN_VP_ADDR);

#if 1
    /* 等待开机动画播放完成: 轮询VP, 值连续3次不变则判定结束 */
    delay_ms(2000);

    while (1) {
        DWIN_ReadVar(ANIM_VP_ADDR, 2);
        delay_ms(200);
        DWIN_ProcessRecvFrame();

        if (g_anim_resp) {
            g_anim_resp = 0;

            if (g_anim_val == prev_val) {
                stable_cnt++;
                if (stable_cnt >= 3)
                    break;
            } else {
                prev_val = g_anim_val;
                stable_cnt = 0;
            }
        }

        delay_ms(300);
    }
#endif

    /* 变量初始化: 在跳转页面1之前写入全部VP初值, 避免跳页后短暂显示旧值/默认值 */
    DWIN_DisplayPumpPower(0);
    DWIN_DisplayPressure(0.0f);
    DWIN_WriteVar(DWIN_VP_ADDR_PREINFUSION, 0);    /* 0xD000预浸泡初值0 */
    DWIN_WriteVar(DWIN_VP_ADDR_CUR_TEMP, 0);       /* 0xD004温度初值0 */
    DWIN_WriteVar(DWIN_VP_ADDR_CUR_PRES, 0);       /* 0xD008压力初值0 */
    DWIN_WriteVar(DWIN_VP_ADDR_BREW_ELAPSED, 0);   /* 0xD00C萃取出水计时初值0 */
    UpdatePreinfuseText();                          /* 刷新 0x8A00/0x8A40/0x8A80 文本 */
    DWIN_WriteVar(0xD04C, g_d04c_val);     /* 灯板开关初值: 0/1 (按断电保存) */
    DWIN_WriteVar(0xD050, g_d050_val);     /* 灯板开关镜像: 与 D04C 一致 */
    DWIN_WriteVar(0xD054, g_d054_val);     /* 灯板亮度档初值: 0-10 (按断电保存) */
    /* 屏幕亮度(0-10 -> 0-100%), 待机亮度同步, 不进待机 */
    DWIN_SetBacklight((u8)(Settings_Get()->brightness * 10u),
                      (u8)(Settings_Get()->brightness * 10u), 0u);
    /* 设定类VP: flash -> 屏幕 (杯数/萃取温度/冲泡头温度/萃取时间/语言) */
    DWIN_WriteVar(SET_VP_CUP_COUNT, Settings_Get()->cup_count);
    DWIN_WriteVar(SET_VP_BREW_TEMP, (uint16_t)(Settings_Get()->brew_temp + 0.5f));
    DWIN_WriteVar(SET_VP_HEAD_TEMP, (uint16_t)(Settings_Get()->head_temp + 0.5f));
    DWIN_WriteVar(SET_VP_BREW_TIME, Settings_Get()->brew_time_s);
    DWIN_WriteVar(SET_VP_LANG, Settings_Get()->lang_mode);
        DWIN_WriteVar(SET_VP_PREINFUSE_DUR,     Settings_Get()->preinfuse_duration_s);
        DWIN_WriteVar(SET_VP_PREINFUSE_STOP,    Settings_Get()->preinfuse_stop_s);
        DWIN_WriteVar(SET_VP_PREINFUSE_ENABLE,  Settings_Get()->preinfuse_master);
        UpdatePreinfuseText();                          /* 刷新 0x8A00/0x8A40/0x8A80 */
    DWIN_WriteVar(0xD250, Settings_Get()->curve_knob_val);  /* 曲线页旋钮初值(flash 持久化) */
    /* 动画结束, 跳转页面1 (初值已就位) */
    Page_SwitchTo(1);
    delay_ms(500);

    /* 主循环: PID温控 + 萃取状态机 + 泵控制 + 显示更新 */
    printf("=== Entering main loop ===\r\n");
    last_pid_tick = GetTick();
    last_display_tick = GetTick();
    last_pres_ctrl_tick = GetTick();

    /* 点击0xD000增量控件触发锅炉流程. 基准值取增量控件量程(5~50)内的下限5 */
    CooldownTest_Init(5);

    /* SEL1 选择控件初始化 (0xD100-0xD118 跳页 5-11) */
    SelPoll_Init();

    Language_Poll(GetTick());   /* 开机应用默认语言(0xD060: 0=中文, 1=英文) */

    while (1) {
        DWIN_ProcessRecvFrame();

        u32 now = GetTick();

        /* SEL1 轮询 (0xD100-0xD118 跳页 5-11) */
        SelPoll_Run(now);
        D218_Curve_Poll(now);   /* 0xD218 0→1 边沿 → 跳页4 + 起曲线, 完后写回 0 */
        D04C_Light_Poll(now);   /* D04C/D050 镜像驱动灯板 PWM, D054 变更写入 settings.light_level */
        SettingsVP_Poll(now);   /* 设定类VP双向同步(变化->flash) */
        Language_Poll(now);

        /* 恢复出厂预设曲线: 每次最多写一页(100ms一页), 全部写完后重画CH1.
         * 出饮过程中不做(每页要关中断~40ms, 会冻结泵/加热的时序). */
        if (g_preset_restore_active && g_bev_state == BEV_IDLE) {
            if (Preset_RestoreAllTask(now)) {
                g_preset_restore_active = 0;
                if (g_on_curve_page) {
                    Preset_Load((uint8_t)Settings_Get()->curve_knob_val);
                    Curve_ApplyXStepByPreset();
                    CURVE_DrawRefFixed(Preset_Data(), Preset_Length(),
                                       PRESET_CURVE_MUL_X100, 1);
                }
            }
        }

        /* 客户手动退出曲线页 (屏端上报 CURVE_EXIT_VP, 不看值):
         *   停止曲线录制 + 结束出饮流程;
         *   若客户改过预设且距第一次扭旋钮已超过15s -> 弹"是否保存"提示,
         *   等用户选择后再处理(此时不跳页); 否则直接回主界面. */
        if (g_curve_exit_pending) {
            g_curve_exit_pending = 0;
            if (g_on_curve_page) {
                g_curve_running = 0;
                /* 退出曲线页: 清掉变压模式标志, 之后在别的页按KEY1是正常萃取
                 * (预浸泡+满压, 泵不跟预设), 否则会残留成"泵被预设曲线驱动" */
                g_curve_drive = 0;
                g_curve_knob_touched = 0;
                CURVE_Pause();
                /* 结束出饮流程 (此时 g_on_curve_page 仍为1, Beverage_Stop 不会跳页) */
                if (g_bev_state != BEV_IDLE) {
                    Beverage_Stop();
                }
                g_on_curve_page = 0;
                if (!Curve_PresetEditOnExit(now)) {
                    /* 无需提示 -> 回主界面 */
                    Beverage_GoHome();
                }
                /* 需提示时: Curve_PresetEditOnExit 已切到弹窗页, 等用户选择 */
            }
            /* 已在曲线页外(屏端重复上报) -> 忽略, 避免二次触发退出流程 */
        }

        /* 保存提示弹窗的用户选择 */
        if (g_preset_choice != 0u) {
            uint8_t ch = g_preset_choice;
            g_preset_choice = 0u;
            Curve_SaveChoiceHandle(ch);
        }

        /* 点击0xD000增量控件 → 启动锅炉流程 */
        CooldownTest_Poll(now);

        #if SYS_UART_CMD_ENABLE
        /* 串口命令 (/extract /hot /steam /help) */
        CmdConsole_Poll();
#endif

        /* 按键状态变化监听: 打印序号+新状态 */
        {
            uint8_t kidx, kst;
            while (KeyMonitor_PopEvent(&kidx, &kst)) {
                printf("Key %u: %s\r\n", (unsigned)(kidx + 1),
                       kst ? "OFF (high)" : "ON (low)");
                if (kst != KEY_STATE_ON) {
                    continue;   /* 只处理按下(低电平) */
                }
                /* 任意面板按键按下: 蜂鸣器响一次 (104ms, 迪文屏扬声器发声) */
                DWIN_BuzzerPlayOnce();
                if (g_bev_state != BEV_IDLE) {
                    /* 出饮流程进行中: 任意按键按下即停止 */
                    printf(">>> Beverage stopped by key %u\r\n",
                           (unsigned)(kidx + 1));
                    Beverage_Stop();
                } else if (g_on_curve_page && kidx == 0) {
                    /* 曲线页 + KEY1: 启动 CH0 实时录制, 不跳页/不开泵阀 */
                    Beverage_CurveStartCH0();
                } else if (kidx == 2) {
                    Beverage_StartSteam();  /* 按键3(PB7): 蒸汽 (曲线页里也允许) */
                } else if (kidx == 0) {
                    Beverage_StartBrew(TEMP_GetLastTemperature()); /* 按键1(PA5): 萃取 */
                } else if (kidx == 1) {
                    Beverage_StartHot();    /* 按键2(PA7): 热水 (曲线页里也允许) */
                }
            }
        }

        /* 旋钮: 旋转时打印累计增量(自上次读取以来累计的格数)
 * 曲线页 + 未启动 CH0 录制 (g_on_curve_page && !g_curve_running):
 *   旋钮 delta 用于调节 0xD250 (settings.curve_knob_val, 0-8 循环).
 *   每格 ±1, 超界循环, 写 settings + 写屏.
 * 曲线页 + 已按 KEY1 启动 CH0 录制 (g_on_curve_page && g_curve_running):
 *   旋钮 delta 用于调节 AC 泵功率 (±5%/格, 0-100% 循环).
 *   调泵后立即 AC_Pump_SetPower + 打印新功率. */
        {
            int32_t d = Encoder_PopDelta();
            if (d != 0) {
                printf("Encoder delta: %+d\r\n", (int)d);
                if (g_on_curve_page && !g_curve_running) {
                    Settings_t *s = Settings_Edit();
                    uint16_t v   = s->curve_knob_val;
                    uint16_t min = SETTINGS_CURVE_KNOB_MIN;
                    uint16_t max = SETTINGS_CURVE_KNOB_MAX;
                    int32_t step = d;
                    /* 把任意 ±N 化为 ±1 多次循环处理 (处理累积 delta) */
                    while (step > 0) {
                        v = (uint16_t)((v >= max) ? min : (v + 1u));
                        step--;
                    }
                    while (step < 0) {
                        v = (uint16_t)((v <= min) ? max : (v - 1u));
                        step++;
                    }
                    if (v != s->curve_knob_val) {
                        s->curve_knob_val = v;
                        Settings_MarkDirty();
                        DWIN_WriteVar(0xD250, v);
                        /* 切换预设槽: 读新槽到 RAM 工作区, 按新槽点数重定步长 + 重画 CH1 */
                        Preset_Load((uint8_t)v);
                        Curve_ApplyXStepByPreset();
                        CURVE_DrawRefFixed(Preset_Data(), Preset_Length(),
                                           PRESET_CURVE_MUL_X100, 1);
                        printf(">>> Curve knob: 0xD250 = %u (preset slot %u loaded)\r\n",
                               (unsigned)v, (unsigned)v);
                    }
                } else if (g_on_curve_page && g_curve_running) {
                    /* 录制中: 旋钮调泵频率 ±1.25Hz/格 (0.25Hz单位 ±5), 0~50Hz 限幅 (到边界停止) */
                    uint16_t cur = AC_Pump_GetFrequency();   /* qhz, 0~200 */
                    int32_t step = d;
                    while (step > 0 && cur < AC_FREQ_MAX_QHZ) {
                        cur = (uint16_t)(cur + AC_FREQ_STEP_QHZ);
                        if (cur > AC_FREQ_MAX_QHZ) cur = AC_FREQ_MAX_QHZ;
                        step--;
                    }
                    while (step < 0 && cur > AC_FREQ_MIN_QHZ) {
                        cur = (uint16_t)(cur - AC_FREQ_STEP_QHZ);
                        step++;
                    }
                    /* 客户扭了旋钮: 脱离预制曲线, 改由旋钮控制.
                     * 第一次扭动时记录时刻(15s判定起点)并标记预设已被改动. */
                    Curve_MarkPresetEdited(now);
                    AC_Pump_SetFrequency(cur);
                    /* 打印频率(Hz): qhz/4, 保留2位小数 (qhz 步进5 → 1.25Hz) */
                    printf(">>> Pump freq: %u.%02u Hz\r\n",
                           (unsigned)(cur / 4u), (unsigned)((cur % 4u) * 25u));
                }
            }
        }

        /* 快速循环: 锅炉加热PID + 阀门控制 (每10ms, 待机等触发) */
        if (now - last_pid_tick >= FAST_LOOP_PERIOD_MS) {
            temperature = TEMP_ReadTemperature();
            OTEMP_ReadTemperature();   /* 出水口温度采样(滤波), 慢速循环取值显示 */
            KeyMonitor_Update(now);    /* 按键扫描+消抖(10ms节拍) */

            /* 锅炉超温保护: 放在锅炉PID之前, 保护中 HEATER_SetPower 只能关不能开 */
            if (temperature > -100.0f) {
                (void)HEATER_CheckProtection(temperature);
            }

            /* 锅炉: PID连续控加热功率 + 温度到位开阀出水 */
            Boiler_Update(temperature, now);

            /* 冲泡头加热片 (PB10): 继电器通断控温.
             * 仅在萃取流程 (HeatStart 后) 生效, 其余流程内部 enabled=0 直接返回. */
            BrewHeater_Update(OTEMP_GetLastTemperature(), now);

            /* 出饮流程推进(热水/萃取): 超时/到温/缺水判断 */
            Beverage_Update(now, temperature);

            last_pid_tick = now;
        }

        /* 压力/功率曲线: CH0 每 250ms 追加一点 (Y 轴=实时功率%, 0-100, 不再是压力).
 * 步长2和步长4都是 250ms 一帧, 区别仅在 X 轴像素步长.
 * 主循环100ms节拍为基础, 累计达到 g_curve_sample_period 才真正调一次 CURVE_DualAppend. */
        if (now - last_pres_ctrl_tick >= g_curve_sample_period) {
            if (g_curve_running && g_curve_drive &&
                (g_curve_ch0_count >= CURVE_REC_MAX_POINTS ||
                 (!g_curve_knob_touched && g_curve_ch0_count >= Preset_Length()))) {
                /* 变压萃取的时长 (设定的萃取时间不参与, 见 Beverage_Update 的 BEV_BREW_RUN):
                 *   没扭旋钮: 跟预设走, 走到结束标志(无效位)结束, 最长 99s;
                 *   扭过旋钮: 客户手动控泵, 不受预设长度限制, 录满 99s 结束 */
                if (g_curve_knob_touched && g_on_curve_page) {
                    /* 手动控泵且一直没自己退出: 按屏上"退出曲线页"处理(下一轮主循环执行) ——
                     * 停萃取, 改动超过 15s 时弹"是否保存" */
                    printf(">>> Curve: %u pts (99s) reached, auto exit\r\n",
                           (unsigned)g_curve_ch0_count);
                    g_curve_exit_pending = 1u;
                } else {
                    printf(">>> Curve: end reached (%u pts), brew done\r\n",
                           (unsigned)g_curve_ch0_count);
                    Beverage_Stop();   /* 停泵关阀, 内部 Beverage_CurveDataStop 停止录制; 留在曲线页 */
                }
            } else if (g_curve_running) {
                uint16_t idx = g_curve_ch0_count;
                uint16_t qhz;
#if CURVE_DIAG_ENABLE
                /* 目标档位必须在下面写实际值之前捕获, 否则读到的是实际值 */
                uint8_t  tgt_gear = (idx < Preset_Length()) ? Preset_Data()[idx] : 0u;
#endif

                /* (1) 变压模式 + 客户未扭旋钮: 水泵频率跟随预制曲线
                 *     -> 实际曲线(CH0)与预制曲线(CH1)完全一致 */
                if (g_curve_drive && !g_curve_knob_touched) {
                    Curve_ApplyPresetPump(idx);
                }

                /* (2) 把实际频率记录到工作区的对应点 (档位 = qhz / 5)
                 *     注意: 必须先读预设再写实际值, 否则会把预设覆盖掉.
                 *     扭过旋钮后会越过预设的结束标志继续录(最长 CURVE_REC_MAX_POINTS 点),
                 *     保存时由 Preset_SetLength 把结束标志放到实际曲线末尾. */
                qhz = AC_Pump_IsOn() ? AC_Pump_GetFrequency() : 0u;
                if (idx < CURVE_REC_MAX_POINTS) {
                    uint8_t gear = (uint8_t)(qhz / PRESET_GEAR_QHZ);
                    if (gear > PRESET_GEAR_MAX) gear = PRESET_GEAR_MAX;
                    Preset_Data()[idx] = gear;
                }

                /* (3) Y 轴值: 水泵频率 (Hz). CURVE_PRES_SCALE=10 → 内部 ×10 → 0~50Hz 映射到 Y 0~500 */
                CURVE_DualAppend((float)qhz / 4.0f, 0.0f);
                g_curve_ch0_count++;

#if CURVE_DIAG_ENABLE
                /* [诊断] 目标档位 vs 实际档位: 正常 tgt == act (泵完全跟随预设).
                 * act 恒为 40档(50Hz) -> 采样器没在设泵(看 drive/knob).
                 * pump=OFF -> 萃取没起来(看 "Brew: tank empty"/"boiler busy").
                 * 每250ms一条, 默认关(太吵且拖慢主循环). */
                {
                    printf(">>> Curve pt%u: tgt=%u act=%u qhz=%u pump=%s drive=%u knob=%u\r\n",
                           (unsigned)idx, (unsigned)tgt_gear,
                           (unsigned)(qhz / PRESET_GEAR_QHZ), (unsigned)qhz,
                           AC_Pump_IsOn() ? "on" : "OFF",
                           (unsigned)g_curve_drive, (unsigned)g_curve_knob_touched);
                }
#endif

                /* 步长兜底: 预设点数<=200(步长4)时, 若实际录到 200 点以上就拉宽到步长2,
                 * 否则 CH0 会超出可视区. 预设本身>200点时步长已经是2, 不需要动.
                 * 两条曲线同步改步长, 保证同一 X 尺度. 切换只发一次. */
                if (!g_curve_xstep_expanded && g_curve_xstep == 4u
                    && g_curve_ch0_count >= 200u) {
                    Curve_ApplyXStep(2u);
                    g_curve_xstep_expanded = 1;
                    printf(">>> Curve: CH0 >= 200 pts, X step 4->2 (sample 250ms unchanged)\r\n");
                }
            }
            last_pres_ctrl_tick = now;
        }

        /* 慢速循环: 屏幕显示 + 压力 (每1秒) */
        if (now - last_display_tick >= 1000) {
            temperature = TEMP_GetLastTemperature();
            pressure = PRES_ReadPressure();

            /* 0xD004显示当前温度(4字节单精度浮点数, 屏显量化到0.5℃) */
            if (temperature > -100.0f) {
                DWIN_WriteVarFloat(DWIN_VP_ADDR_CUR_TEMP, QuantTemp_ForScreen(temperature));
            }
            /* 0xD008显示锅炉压力(4字节单精度浮点数) */
            DWIN_WriteVarFloat(DWIN_VP_ADDR_CUR_PRES, pressure);
            /* 0xD010显示冲泡头(出水口)温度(4字节单精度浮点数, 屏显量化到0.5℃) */
            DWIN_WriteVarFloat(DWIN_VP_ADDR_HEAD_TEMP,
                               QuantTemp_ForScreen(OTEMP_GetLastTemperature()));
            DWIN_DisplayPressure(pressure);
            DWIN_DisplayPumpPower((uint8_t)(AC_Pump_GetPower() / 10));   /* 0.1% → 整数% */

            /* 0xD00C 萃取出水后经过时间(秒, 1秒刷新, BEV_BREW_RUN 期间有效) */
            if (g_bev_state == BEV_BREW_RUN) {
                uint16_t elapsed_s = (uint16_t)((now - g_bev_start_tick) / 1000u);
                DWIN_WriteVar(DWIN_VP_ADDR_BREW_ELAPSED, elapsed_s);
            }

            if (temperature > -100.0f) {
                DWIN_DisplayTemperature(temperature);
                DWIN_DisplayPower(HEATER_GetPower());
            }

            printf("Temp: %.1f C, Outlet: %.1f C, State: %d, Heat: %d%%, Valve: %s, HotValve: %s, Pres: %.2f Bar, Water: %s\r\n",
                   temperature, OTEMP_GetLastTemperature(), (int)Boiler_GetState(), Boiler_GetHeaterPower(),
                   Boiler_IsValveOpen() ? "OPEN" : "CLOSE",
                   Boiler_HotValveIsOpen() ? "OPEN" : "CLOSE", pressure,
                   WATER_IsPresent() ? "OK" : "LOW");

            /* 萃取加热片 (PB10 继电器通断) 控温状态, 便于观察.
             * 仅萃取流程有输出, 其余流程继电器保持断开. */
            if (g_bev_state == BEV_BREW_PREINFUSE ||
                g_bev_state == BEV_BREW_PREINFUSE_STOP ||
                g_bev_state == BEV_BREW_RUN) {
                float bh_duty, bh_low, bh_high;
                uint8_t bh_on;
                BrewHeater_GetTerms(&bh_duty, &bh_on, &bh_low, &bh_high);
                printf("  [BrewHeat] head=%.1f (on<%.1f, off>=%.1f target=%.0f) | relay=%s\r\n",
                       OTEMP_GetLastTemperature(), bh_low, bh_high,
                       Settings_Get()->head_temp,
                       bh_on ? "ON" : "OFF");
            }

            /* 客户设定延迟落盘: 修改静置5s后写一次Flash末页 */
            Settings_Task(now);

            last_display_tick = now;
        }
    }

}
