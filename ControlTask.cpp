#include "ControlTask.h"

#include "config.h"
#include "Gm6020.h"
#include "Pid.h"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"

#include <cmath>

namespace {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kTwoPi   = 2.0f * kPi;
constexpr float kRad2Deg = 180.0f / kPi;

// 控制对象
Gm6020 g_motor(MOTOR_ID);

// PID 控制器
Pid g_speedPid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,
               SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, SPEED_PID_OUT_MIN);
Pid g_posPid(POS_PID_KP, POS_PID_KI, POS_PID_KD,
             POS_PID_I_MAX, POS_PID_OUT_MAX, POS_PID_OUT_MIN);

// 1kHz 节拍计数。CAN 中断只写 g_lastFeedbackTick，两个变量均为 32 位。
uint32_t g_tick = 0;
volatile uint32_t g_lastFeedbackTick = 0;
volatile bool g_feedbackReceived = false;

// 位置模式：上电位置 home（用于围绕 home 做正弦摆动）
float    g_posHomeDeg = 0.0f;
bool     g_homeLatched = false;

// 喂狗
void FeedWatchdog(void) {
#if WATCHDOG_ENABLE
    HAL_IWDG_Refresh(&hiwdg);
#endif
}

// 发送单电机电流控制帧。GM6020 的电流给定范围是 -16384~16384。
void SendCurrentCmd(void) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t  tx_data[8] = {0};
    uint32_t mailbox;

    g_motor.encode(tx_data);

    tx_header.StdId = g_motor.txId();
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

bool FeedbackIsFresh(void) {
    if (!g_feedbackReceived) return false;
    return (g_tick - g_lastFeedbackTick) <= CAN_FEEDBACK_TIMEOUT_TICKS;
}

} // namespace

// ---------------------------------------------------------------------------
// Ozone Timeline 观测变量
//
// 这些变量放在全局作用域（而不是上面的匿名 namespace），Ozone 才能在
// Timeline / Data Sampling 窗口中直接按名字添加：
//   g_tim_vel_curve_rpm  速度正弦曲线给定          (rpm)
//   g_tim_vel_ref_rpm    速度环实际目标            (rpm)
//                        速度模式 = 速度正弦给定
//                        位置模式 = 位置环输出的期望转速
//   g_tim_vel_act_rpm    实际转速                  (rpm)
//   g_tim_pos_ref_deg    位置正弦给定（相对 home）  (deg, 以 0 为中心)
//   g_tim_pos_act_deg    实际位置（相对 home）      (deg, 以 0 为中心)
//   g_tim_friction_ff    摩擦补偿前馈电流          (电流码值)
//
// volatile 保证 1kHz 中断里的写入不会被编译器优化掉，Ozone 可实时采样。
// ---------------------------------------------------------------------------
extern "C" {
volatile float g_tim_vel_curve_rpm = 0.0f;
volatile float g_tim_vel_ref_rpm   = 0.0f;
volatile float g_tim_vel_act_rpm   = 0.0f;
volatile float g_tim_pos_ref_deg   = 0.0f;
volatile float g_tim_pos_act_deg   = 0.0f;
volatile float g_tim_friction_ff   = 0.0f;
}

namespace {

// 摩擦补偿前馈：库仑摩擦 + 粘滞摩擦。
// 以速度环目标 v_ref 判断运动方向；tanh 让库仑项在零速附近平滑过零。
float FrictionFeedforward(float vel_ref_rpm) {
#if FRICTION_FF_ENABLE
    const float sign_scale =
        (FRICTION_FF_SIGN_SCALE_RPM > 0.0f) ? FRICTION_FF_SIGN_SCALE_RPM : 1.0f;
    const float sign = std::tanh(vel_ref_rpm / sign_scale);
    return FRICTION_FF_COULOMB * sign + FRICTION_FF_VISCOUS * vel_ref_rpm;
#else
    (void)vel_ref_rpm;
    return 0.0f;
#endif
}

// 速度环 + 摩擦前馈，并把叠加后的结果限制在电流指令范围内。
float SpeedLoopOutput(float ref_vel_rpm, float act_vel_rpm) {
    float out = g_speedPid.calc(ref_vel_rpm, act_vel_rpm);
    const float ff = FrictionFeedforward(ref_vel_rpm);
    g_tim_friction_ff = ff;

    out += ff;
    if (out > SPEED_PID_OUT_MAX) out = SPEED_PID_OUT_MAX;
    if (out < SPEED_PID_OUT_MIN) out = SPEED_PID_OUT_MIN;
    return out;
}

} // namespace

extern "C" void ControlTaskInit(void) {
    // 1. 配置 CAN 过滤器：全零掩码放行消息到 FIFO0，再由回调匹配电机 ID。
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0;
    filter.SlaveStartFilterBank  = 14;  // CAN1 使用 0~13，CAN2 使用 14~27
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // 2. 启动 CAN 并开启接收中断
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    g_tick = 0;
    g_lastFeedbackTick = 0;
    g_feedbackReceived = false;
    g_speedPid.reset();
    g_posPid.reset();

    // 3. 先喂一次狗，再启动 1kHz 控制定时器；此喂狗方式不能检测主循环卡死。
    FeedWatchdog();
    HAL_TIM_Base_Start_IT(&htim6);
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == g_motor.rxId()) {
        g_motor.decode(data);
        g_lastFeedbackTick = g_tick;
        g_feedbackReceived = true;
    }
}

extern "C" void MainTask(void) {
    FeedWatchdog();

    g_tick++;
    float t = static_cast<float>(g_tick) * CONTROL_PERIOD_S;

    // 先更新实际量，反馈超时/断线时 Timeline 仍能看到最后一次反馈值。
    // 位置实际量使用相对 home 的角度，不再把累计绝对角度直接显示到 Timeline。
    const float pos_abs_deg = g_motor.angle() * kRad2Deg;
    g_tim_vel_act_rpm = g_motor.velRpm();
    g_tim_pos_act_deg = pos_abs_deg - g_posHomeDeg;

    // 反馈超时时清空 PID 并尝试发送零电流；CAN 断线时不保证指令能送达。
    if (!FeedbackIsFresh()) {
        g_speedPid.reset();
        g_posPid.reset();
        g_tim_friction_ff = 0.0f;   // 超时分支不叠加前馈
        g_motor.setCurrent(0);
        SendCurrentCmd();
        return;
    }

    // 上电后第一次有效反馈的位置作为位置曲线的 home。
    // 速度模式也锁存，这样 Timeline 里的位置目标/实际量使用同一基准。
    if (!g_homeLatched) {
        g_posHomeDeg  = pos_abs_deg;
        g_homeLatched = true;
        g_tim_pos_act_deg = 0.0f;   // 锁存当拍，实际位置就是 home
    }

    // 两条正弦曲线都按配置计算，便于在 Ozone 中同时观察目标与实际。
    // v(t)    = SPEED_SINE_AMP_RPM * sin(2*pi*SPEED_SINE_FREQ_HZ*t)
    // p_abs(t)= home + POS_SINE_AMP_DEG * sin(2*pi*POS_SINE_FREQ_HZ*t)
    // Timeline 里的位置目标使用相对 home 的 p_rel(t)，与相对位置实际量对齐。
    const float ref_vel_curve_rpm =
        SPEED_SINE_AMP_RPM * std::sinf(kTwoPi * SPEED_SINE_FREQ_HZ * t);
    const float ref_pos_rel_deg =
        POS_SINE_AMP_DEG * std::sinf(kTwoPi * POS_SINE_FREQ_HZ * t);
    const float ref_pos_curve_deg = g_posHomeDeg + ref_pos_rel_deg;

    g_tim_vel_curve_rpm = ref_vel_curve_rpm;
    g_tim_pos_ref_deg   = ref_pos_rel_deg;

    if (CONTROL_MODE == 0) {
        // ---- 速度正弦跟随：v(t) = A * sin(2*pi*f*t) ----
        g_tim_vel_ref_rpm = ref_vel_curve_rpm;
        float out = SpeedLoopOutput(g_tim_vel_ref_rpm, g_tim_vel_act_rpm);
        g_motor.setCurrent(static_cast<int16_t>(out));
    } else {
        // ---- 位置正弦跟随（串级：位置环 -> 速度环）----
        // 外环：位置环输出期望速度（rpm）
        // 控制仍使用连续绝对角度 pos_abs_deg，Timeline 只显示相对 home 的值。
        float ref_vel_rpm = g_posPid.calc(ref_pos_curve_deg, pos_abs_deg);
        g_tim_vel_ref_rpm = ref_vel_rpm;

        // 内环：速度环输出电流码值，并叠加摩擦补偿前馈
        float out = SpeedLoopOutput(ref_vel_rpm, g_tim_vel_act_rpm);
        g_motor.setCurrent(static_cast<int16_t>(out));
    }

    SendCurrentCmd();
}

// ---------------------------------------------------------------------------
// HAL 弱回调实现（覆盖 HAL 库中的弱定义）
// ---------------------------------------------------------------------------
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan != &hcan1 || HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }
    CanFeedbackCallback(rx_header.StdId, rx_data);
}
