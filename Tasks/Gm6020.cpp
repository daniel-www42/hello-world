#include "Gm6020.h"

namespace {
constexpr float    kPi         = 3.14159265358979323846f;
constexpr uint16_t kAngleRange = 8192;                // 机械角度一圈 8192
constexpr float    kRpmToRadS  = (2.0f * kPi) / 60.0f;
constexpr float    kRawToAmp   = 3.0f / 16384.0f;     // 反馈电流 ±16384 对应 ±3A（近似）
}

Gm6020::Gm6020(uint8_t id)
    : id_(id), voltage_(0),
      raw_angle_(0), raw_vel_rpm_(0), raw_current_(0), raw_temp_(0),
      full_turns_(0), last_raw_angle_(0), feedback_valid_(false) {}

uint32_t Gm6020::rxId() const {
    return 0x204u + id_;   // 0x205 ~ 0x20B
}

uint32_t Gm6020::txId() const {
    return (id_ <= 4) ? 0x1FEu : 0x2FEu;   // 保留原工程已实测可用的电流控制 ID
}

void Gm6020::decode(const uint8_t *data) {
    uint16_t raw = static_cast<uint16_t>((data[0] << 8) | data[1]);

    // 第一次反馈只建立初值，不能把上电时的绝对角度当成跨圈。
    if (feedback_valid_) {
        // 跨零检测：一圈 8192，相邻两次采样跳变超过半圈（4096）视为跨过零点。
        int16_t diff = static_cast<int16_t>(raw - last_raw_angle_);
        if (diff > 4096)       full_turns_--;  // 0 -> 8191，反向跨零
        else if (diff < -4096) full_turns_++;  // 8191 -> 0，正向跨零
    }

    last_raw_angle_ = raw;
    raw_angle_      = raw;
    raw_vel_rpm_    = static_cast<int16_t>((data[2] << 8) | data[3]);
    raw_current_    = static_cast<int16_t>((data[4] << 8) | data[5]);
    raw_temp_       = data[6];
    feedback_valid_ = true;
}

float Gm6020::angle() const {
    // 累计角度 = 整圈数 * 一圈 + 当前原始角度，再换算成弧度
    return (static_cast<float>(full_turns_) * kAngleRange + raw_angle_)
           * (2.0f * kPi / kAngleRange);
}

float Gm6020::vel() const {
    return raw_vel_rpm_ * kRpmToRadS;   // rpm -> rad/s
}

float Gm6020::current() const {
    return raw_current_ * kRawToAmp;
}

float Gm6020::temp() const {
    return static_cast<float>(raw_temp_);
}

int16_t Gm6020::velRpm() const {
    return raw_vel_rpm_;
}

void Gm6020::setCurrent(int16_t current) {
    if (current > 16384) current = 16384;
    if (current < -16384) current = -16384;
    voltage_ = current;
}

int16_t Gm6020::currentCommand() const {
    return voltage_;
}

void Gm6020::encode(uint8_t *data) const {
    // 电流控制帧数据布局：
    //   0x1FE：ID 1 -> [0,1]  2 -> [2,3]  3 -> [4,5]  4 -> [6,7]
    //   0x2FE：ID 5 -> [0,1]  6 -> [2,3]  7 -> [4,5]
    uint8_t index = (id_ <= 4) ? static_cast<uint8_t>((id_ - 1) * 2)
                               : static_cast<uint8_t>((id_ - 5) * 2);
    data[index]     = static_cast<uint8_t>((voltage_ >> 8) & 0xFF);
    data[index + 1] = static_cast<uint8_t>(voltage_ & 0xFF);
}
