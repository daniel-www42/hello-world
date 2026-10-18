#include "Pid.h"

Pid::Pid(float kp, float ki, float kd,
         float i_max, float out_max, float out_min)
    : kp_(kp), ki_(ki), kd_(kd),
      i_max_(i_max), out_max_(out_max), out_min_(out_min),
      integ_(0.0f), last_err_(0.0f) {}

void Pid::setParams(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}

void Pid::setLimit(float i_max, float out_max, float out_min) {
    i_max_ = i_max; out_max_ = out_max; out_min_ = out_min;
}

void Pid::reset() {
    integ_ = 0.0f;
    last_err_ = 0.0f;
}

float Pid::calc(float ref, float fdb) {
    float err = ref - fdb;

    float p = kp_ * err;

    integ_ += ki_ * err;
    if (integ_ > i_max_)  integ_ = i_max_;
    if (integ_ < -i_max_) integ_ = -i_max_;

    float d = kd_ * (err - last_err_);
    last_err_ = err;

    float out = p + integ_ + d;
    if (out > out_max_) out = out_max_;
    if (out < out_min_) out = out_min_;

    return out;
}
