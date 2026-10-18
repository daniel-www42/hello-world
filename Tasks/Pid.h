#ifndef PID_H
#define PID_H

// 位置式 PID 控制器
//
// 离散公式（控制周期固定 1kHz，Ki 已隐含采样周期 T）：
//   e(k)     = ref - fdb
//   P(k)     = Kp * e(k)
//   I(k)     = I(k-1) + Ki * e(k)          （积分项，限幅 i_max）
//   D(k)     = Kd * (e(k) - e(k-1))        （微分项）
//   out(k)   = P(k) + I(k) + D(k)          （输出限幅）
//
// 抗积分饱和：积分项独立限幅 + 输出限幅
class Pid {
public:
    Pid(float kp, float ki, float kd,
        float i_max, float out_max, float out_min);
    ~Pid() = default;

    void setParams(float kp, float ki, float kd);
    void setLimit(float i_max, float out_max, float out_min);
    void reset();

    // 计算一次控制量：ref 目标值，fdb 反馈值，返回限幅后的输出
    float calc(float ref, float fdb);

private:
    float kp_, ki_, kd_;
    float i_max_;
    float out_max_, out_min_;
    float integ_;      // 积分项
    float last_err_;   // 上一拍误差 e(k-1)
};

#endif // PID_H
