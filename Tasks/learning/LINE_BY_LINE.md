# Tasks 源码逐行讲解

基于本次读取的源码：`ControlTask.cpp` 166 行、`Gm6020.cpp` 82 行、`Pid.cpp` 39 行；头文件和配置见各节。未改动控制代码。后续改源码后，行号可能变化，应同时按函数名和语句核对。

先读 [概念与实践讲义](README.md)。这份文件用于查行，不要求一次背完。

每次随机挑一行，先自己说：这是什么语法？读了什么数据？改了什么状态？单位是什么？下一个步骤谁使用结果？

## 通用行

所有空行只为排版；所有 `//` 注释不参与执行，可能过时，以真实语句为准。单独的 `{`、`}` 表示作用域或语句块起止；类结尾 `};` 还需要分号。后文表格逐项覆盖有实际含义的代码和关键注释，重复的空行和括号按本段解释。

## Pid.cpp

建议最先读这个实现，它只有 39 行，控制计算最集中。

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1 | `#include "Pid.h"` | 引入 Pid 类声明，让编译器知道成员和接口。不是运行时调用。 |
| 3 | `Pid::Pid(float kp, float ki, float kd,` | 定义构造函数，名称与类相同，没有返回类型。前三个参数是控制器增益。 |
| 4 | `float i_max, float out_max, float out_min)` | 参数列表续行，分别是积分项幅度上限、总输出上限和下限。 |
| 5 | `: kp_(kp), ki_(ki), kd_(kd),` | 初始化列表，把传入参数保存到对象成员。左边带下划线的是成员，括号内是参数。 |
| 6 | `i_max_(i_max), out_max_(out_max), out_min_(out_min),` | 保存三种限幅值。不同 Pid 对象保存各自的参数。 |
| 7 | `integ_(0.0f), last_err_(0.0f) {}` | 积分项和历史误差初始为零；构造函数体为空，因为初始化列表已经完成工作。 |
| 9 | `void Pid::setParams(float kp, float ki, float kd)` | 定义改增益的接口，不返回值。当前控制任务没有调用它，增益来自构造时的宏。 |
| 10 | `kp_ = kp; ki_ = ki; kd_ = kd;` | 三条赋值保存在同一行；改变增益不会自动清除历史积分。 |
| 13 | `void Pid::setLimit(...)` | 定义修改限幅的接口，当前任务也没有动态调用它。 |
| 14 | `i_max_=...; out_max_=...; out_min_=...;` | 存储新限幅。函数没有检查上下限是否合理，也没有立刻重新限制已有积分。 |
| 17 | `void Pid::reset()` | 清除控制器历史，不改增益和限幅。 |
| 18 | `integ_ = 0.0f;` | 清积分记忆。失联后不沿用旧的积分输出。 |
| 19 | `last_err_ = 0.0f;` | 清上一拍误差。当前 D 为零，仍保留此状态供通用 PID 使用。 |
| 22 | `float Pid::calc(float ref, float fdb)` | 目标与反馈作为输入，返回本次输出。两输入必须使用同一单位。 |
| 23 | `float err = ref - fdb;` | 目标减实际，得到当前误差。速度环是 rpm，位置环是度。 |
| 25 | `float p = kp_ * err;` | 当前误差乘比例增益，得到 P 输出。输出单位由该环设计决定。 |
| 27 | `integ_ += ki_ * err;` | 当前误差经 Ki 加权后累加到历史积分；该 Ki 已按固定周期定义。不是每次从零求积分。 |
| 28 | `if (integ_ > i_max_) integ_ = i_max_;` | 正积分超过上限就截断到上限。 |
| 29 | `if (integ_ < -i_max_) integ_ = -i_max_;` | 负积分超过负向幅度就截断到负上限。两行构成对称积分限幅。 |
| 31 | `float d = kd_ * (err - last_err_);` | 用当前与上一拍误差的差计算 D。Kd 已吸收 1/T；当前配置 Kd=0，所以 D=0。 |
| 32 | `last_err_ = err;` | 保存本拍误差，下一拍再作为历史误差使用。必须在本拍 D 计算之后更新。 |
| 34 | `float out = p + integ_ + d;` | 将三部分相加，得到位置式 PID 的完整输出，而非输出增量。 |
| 35 | `if (out > out_max_) out = out_max_;` | 超过正向总输出上限就限制。 |
| 36 | `if (out < out_min_) out = out_min_;` | 低于负向总输出下限就限制。 |
| 38 | `return out;` | 把控制结果交给调用者。该函数不发 CAN，也不直接操作电机硬件。 |

追问：为什么 `integ_` 是成员而 `p` 是局部变量？积分需要跨调用保存历史；P 每次仅由当前误差计算，不需要记忆。

追问：`calc` 能保证电机稳定吗？不能，算法形式正确只是条件之一，参数、负载、符号、周期、延迟和限幅都会影响闭环稳定性。

## Pid.h

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1 | `#ifndef PID_H` | 如果尚未定义 PID_H，才继续展开本头文件。 |
| 2 | `#define PID_H` | 定义保护宏，避免同一翻译单元重复包含导致重复定义。 |
| 4-13 | PID 公式注释 | 解释位置式 PID 的离散计算。周期也应理解为吸收到 Kd 中；这里只做积分限幅与输出限幅。 |
| 14 | `class Pid {` | 声明一个控制器类型，包含参数、历史状态和操作函数。 |
| 15 | `public:` | 后面的接口允许类外调用。 |
| 16-17 | `Pid(float kp, ... float out_min);` | 声明构造函数的六个参数，实现在 Pid.cpp。末尾分号表示这是声明。 |
| 18 | `~Pid() = default;` | 使用默认析构函数。这里没有动态分配资源需要专门释放。 |
| 20 | `void setParams(...)` | 声明调增益接口。 |
| 21 | `void setLimit(...)` | 声明调限幅接口。 |
| 22 | `void reset();` | 声明清历史接口。 |
| 24-25 | `float calc(float ref, float fdb);` | 声明计算接口，返回浮点输出。 |
| 27 | `private:` | 后面的数据由类内部管理，外部不能直接访问。 |
| 28 | `float kp_, ki_, kd_;` | 保存三种增益；一行声明三个 float 成员。 |
| 29 | `float i_max_;` | 积分输出项的对称幅度上限。 |
| 30 | `float out_max_, out_min_;` | 总输出的上限和下限。 |
| 31 | `float integ_;` | 已经乘入 Ki 的积分输出项，需要跨周期保留。 |
| 32 | `float last_err_;` | 上一次 calc 的误差，用于本次 D。 |
| 33 | `};` | 类定义结束。 |
| 35 | `#endif` | 结束头文件保护条件。 |

## Gm6020.cpp

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1 | `#include "Gm6020.h"` | 引入电机类声明。 |
| 3 | `namespace {` | 匿名命名空间，里面的常量只供当前源文件使用。 |
| 4 | `constexpr float kPi=...f;` | 保存单精度 π，constexpr 表示可在编译期求值。更多小数位不等于 float 能保留同样多有效位。 |
| 5 | `constexpr uint16_t kAngleRange=8192;` | 编码器一圈有 8192 个刻度，原始值范围 0..8191。 |
| 6 | `kRpmToRadS=(2.0f*kPi)/60.0f;` | rpm 转 rad/s 的比例：一圈 2πrad，一分钟 60s。 |
| 7 | `kRawToAmp=3.0f/16384.0f;` | 原代码采用的反馈电流估算比例；不是控制环必须使用的量，也应核对实际反馈标度。 |
| 10 | `Gm6020::Gm6020(uint8_t id)` | 构造电机对象，传入电机拨码 ID。这里没有自动检查 ID 是否在 1..7。 |
| 11 | `: id_(id), voltage_(0),` | 保存 ID，电流命令初始为 0。voltage_ 是遗留命名，实际存电流给定。 |
| 12 | `raw_angle_(0), raw_vel_rpm_(0), raw_current_(0), raw_temp_(0),` | 反馈字段初值为零；不代表真测到电机在零位且静止。 |
| 13 | `full_turns_(0), last_raw_angle_(0), feedback_valid_(false) {}` | 圈数、上一角度初始为零；尚未收到反馈，因此不能做跨零判断。 |
| 15 | `uint32_t Gm6020::rxId() const` | 返回反馈报文 ID；末尾 const 表示不修改对象普通成员。 |
| 16 | `return 0x204u + id_;` | 电机 ID=2 时返回 0x206。u 是无符号整数字面量后缀。 |
| 19 | `uint32_t Gm6020::txId() const` | 返回控制报文组 ID。 |
| 20 | `return (id_<=4)?0x1FEu:0x2FEu;` | 三目运算：ID 1..4 用第一组，5..7 用第二组，沿用原电流协议。 |
| 23 | `void Gm6020::decode(const uint8_t *data)` | 从字节数组解析一帧反馈。不允许通过 data 修改字节；指针本身不含长度。 |
| 24 | `uint16_t raw=static_cast<uint16_t>((data[0]<<8) \| data[1]);` | 高字节左移 8 位，与低字节按位或，得到无符号角度编码。 |
| 26-27 | `if (feedback_valid_)` | 仅在已有上一帧真实反馈时做跨圈检测。 |
| 28-29 | `int16_t diff=static_cast<int16_t>(raw-last_raw_angle_);` | 计算当前原始角度减上一帧原始角度，可正可负。 |
| 30 | `if (diff>4096) full_turns_--;` | 例如 2 到 8190，表面大幅增加，解释为反向跨零，累计圈数减一。 |
| 31 | `else if (diff<-4096) full_turns_++;` | 例如 8190 到 2，表面大幅减小，解释为正向跨零，累计圈数加一。 |
| 34 | `last_raw_angle_=raw;` | 保存为下一帧的上一角度。 |
| 35 | `raw_angle_=raw;` | 更新供读取接口使用的当前原始角度。 |
| 36 | `raw_vel_rpm_=static_cast<int16_t>((data[2]<<8) \| data[3]);` | 解出有符号转速，协议单位 rpm。FF 9C 在当前工具链下得到 -100。 |
| 37 | `raw_current_=static_cast<int16_t>((data[4]<<8) \| data[5]);` | 解出有符号电流反馈原始码值。 |
| 38 | `raw_temp_=data[6];` | 读取温度字节，不需要拼接。 |
| 39 | `feedback_valid_=true;` | 标记已经有初始反馈；不是持续在线的保证。 |
| 42 | `float Gm6020::angle() const` | 读取连续累计机械角度，单位 rad。 |
| 43-44 | `return (static_cast<float>(full_turns_)*kAngleRange+raw_angle_)` | 累计圈数乘每圈刻度，再加本圈刻度。先转 float 避免把乘法作为整型累积结果计算。 |
| 45 | `*(2.0f*kPi/kAngleRange);` | 将累计刻度转换为弧度，续接上一行 return。 |
| 48 | `float Gm6020::vel() const` | 声明读取 rad/s 速度的实现。当前控制流程使用 velRpm()，没用这个接口。 |
| 49 | `return raw_vel_rpm_*kRpmToRadS;` | 把 rpm 转换成 rad/s。 |
| 52 | `float Gm6020::current() const` | 读取估算安培电流。当前 PID 不依赖这个接口。 |
| 53 | `return raw_current_*kRawToAmp;` | 电流原始码值乘估算比例。 |
| 56 | `float Gm6020::temp() const` | 读取温度，以 float 返回。 |
| 57 | `return static_cast<float>(raw_temp_);` | 将整数字节转为浮点值，温度单位仍来自协议。 |
| 60 | `int16_t Gm6020::velRpm() const` | 读取转速原始整数 rpm，供速度环使用。 |
| 61 | `return raw_vel_rpm_;` | 不进行额外单位换算。 |
| 64 | `void Gm6020::setCurrent(int16_t current)` | 设置电流给定码值，尚未发送报文。注意这里的 current 是参数，不是 current() 反馈接口。 |
| 65 | `if (current>16384) current=16384;` | 正向协议限幅。 |
| 66 | `if (current<-16384) current=-16384;` | 负向协议限幅。 |
| 67 | `voltage_=current;` | 将限幅后的电流命令写入成员，旧名字不改变协议。 |
| 70 | `int16_t Gm6020::currentCommand() const` | 返回已保存的电流命令。它与实际反馈电流不是一个量。 |
| 71 | `return voltage_;` | 返回 RAM 中的当前给定码值。 |
| 74 | `void Gm6020::encode(uint8_t *data) const` | 将本电机命令写到调用者的数组中。const 约束对象，不妨碍修改 data 指向的外部数组。 |
| 75-77 | 数据布局注释 | 说明每组报文中电机和字节的对应关系。 |
| 78 | `uint8_t index=(id_<=4)?static_cast<uint8_t>((id_-1)*2)` | 第一组计算高字节下标。ID=2 得到 2；每个电机占两字节。 |
| 79 | `:static_cast<uint8_t>((id_-5)*2);` | 三目表达式的第二分支，ID=5 对应第二组下标 0。 |
| 80 | `data[index]=static_cast<uint8_t>((voltage_>>8)&0xFF);` | 取 16 位命令的高 8 位，按协议先放高字节。 |
| 81 | `data[index+1]=static_cast<uint8_t>(voltage_&0xFF);` | 取低 8 位，写入相邻字节。 |

追问：`decode` 为什么不返回反馈结构体？当前设计把解析后的反馈直接保存在电机对象里，其他函数通过读取接口获取；这是实现选择，不是 CAN 的规定。

追问：整数转成 float、float 再转整数会怎样？前者用于连续数值运算，后者会丢掉小数部分；不能假设转换能自动做限幅或合法性检查。

## Gm6020.h

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1-2 | `#ifndef/#define GM6020_H` | 头文件保护，避免重复定义类。 |
| 4 | `#include <cstdint>` | 引入固定位宽整数类型，避免依赖普通 int 在不同平台的位宽。 |
| 6-18 | 协议注释 | 当前电流帧、反馈帧和数据布局概览，与电机手册对应。 |
| 19 | `class Gm6020 {` | 定义电机反馈状态和协议编解码接口，不是电机驱动器内部程序。 |
| 20 | `public:` | 外部可用的构造、读反馈和编码等接口。 |
| 21-22 | `explicit Gm6020(uint8_t id);` | 只能明确构造电机对象，避免整数被无意隐式转换成电机。 |
| 23 | `~Gm6020()=default;` | 默认析构，无需专门释放动态资源。 |
| 25-26 | `uint32_t rxId() const;` | 声明读取反馈报文 ID。 |
| 27 | `uint32_t txId() const;` | 声明读取电流控制组 ID。 |
| 29-30 | `float angle() const;` | 读取累计机械角度，rad。 |
| 31 | `float vel() const;` | 读取速度，rad/s。 |
| 32 | `float current() const;` | 读取换算后的反馈电流估算，A。 |
| 33 | `float temp() const;` | 读取温度，℃。 |
| 34 | `int16_t velRpm() const;` | 读取整数转速，rpm。 |
| 36-37 | `void setCurrent(int16_t current);` | 设置电流给定码值，不是设定 rpm。 |
| 38 | `int16_t currentCommand() const;` | 读取命令值，区别于 current() 的反馈。 |
| 40 | `bool hasFeedback() const { return feedback_valid_; }` | 头文件内直接定义短函数，表示是否至少收到过一帧。当前控制任务另用 g_feedbackReceived 判断。 |
| 42-43 | `void decode(const uint8_t *data);` | 声明反馈解析接口，不能通过此指针修改原数据。 |
| 44 | `void encode(uint8_t *data) const;` | 声明打包接口。注释中的“电压”过时，当前打包的是电流。 |
| 46 | `private:` | 将内部状态隐藏在类中。 |
| 47 | `uint8_t id_;` | 电机拨码 ID，范围应为 1..7。 |
| 48 | `int16_t voltage_;` | 遗留名字，当前实际保存待发送的电流给定码值。 |
| 50 | `uint16_t raw_angle_;` | 当前单圈原始角度，0..8191。 |
| 51 | `int16_t raw_vel_rpm_;` | 当前反馈转速，允许正负。 |
| 52 | `int16_t raw_current_;` | 电流反馈原始码值，允许正负。 |
| 53 | `uint8_t raw_temp_;` | 温度原始字节。 |
| 55 | `int32_t full_turns_;` | 跨零累计圈数，可负。不是直接由电机回传的圈数。 |
| 56 | `uint16_t last_raw_angle_;` | 跨零检测所需的上一帧单圈角度。 |
| 57 | `bool feedback_valid_;` | 区分“还没有首帧”和“已有上一帧”，防止首次误判。 |
| 58 | `};` | 类定义结束。 |
| 60 | `#endif` | 头文件保护结束。 |

## ControlTask.cpp

这份文件负责把“时间、目标、反馈、PID、CAN、保护”串起来，不能只逐行背语法而不知道执行顺序。

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1 | `#include "ControlTask.h"` | 引入本文件对外接口声明。 |
| 3 | `#include "config.h"` | 引入模式、轨迹、PID 和保护参数宏。 |
| 4 | `#include "Gm6020.h"` | 引入电机对象类型。 |
| 5 | `#include "Pid.h"` | 引入 PID 类型。 |
| 7 | `#include "main.h"` | 引入 HAL 基础声明和 Error_Handler 等工程接口。 |
| 8 | `#include "can.h"` | 引入 hcan1 等 CAN 句柄声明。 |
| 9 | `#include "tim.h"` | 引入 htim6 等定时器句柄声明。 |
| 10 | `#include "iwdg.h"` | 引入 hiwdg 等看门狗句柄声明。 |
| 12 | `#include <cmath>` | 引入正弦等数学函数声明。 |
| 14 | `namespace {` | 将内部状态和辅助函数限制在本翻译单元。 |
| 16 | `constexpr float kPi=...f;` | 单精度 π 常量。 |
| 17 | `constexpr float kTwoPi=2.0f*kPi;` | 一周对应 2πrad，正弦相位使用它。 |
| 18 | `constexpr float kRad2Deg=180.0f/kPi;` | 弧度到度的比例。位置环输入按度设计。 |
| 20-21 | `Gm6020 g_motor(MOTOR_ID);` | 构造一个电机对象，当前 ID=2，供发送、解码和反馈读取共用。 |
| 23-24 | `Pid g_speedPid(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD,` | 构造速度 PID，当前参数实际启用 P 和 I。 |
| 25 | `SPEED_PID_I_MAX, SPEED_PID_OUT_MAX, SPEED_PID_OUT_MIN);` | 积分限幅 ±140，总输出限幅 ±16384，输出单位是电流给定码值。 |
| 26 | `Pid g_posPid(POS_PID_KP, POS_PID_KI, POS_PID_KD,` | 构造独立的位置 PID 对象，当前仅 P 非零。两环历史状态相互独立。 |
| 27 | `POS_PID_I_MAX, POS_PID_OUT_MAX, POS_PID_OUT_MIN);` | 位置环总输出限幅 ±300rpm；积分增益为零，积分限幅暂不起实际作用。 |
| 29-30 | `uint32_t g_tick=0;` | 记录已执行的控制节拍。CAN 接收也会读取它，当前注释没有完整描述共享关系。 |
| 31 | `volatile uint32_t g_lastFeedbackTick=0;` | 保存最近一次目标电机反馈对应的控制计数，volatile 提醒编译器它可能被中断更新。 |
| 32 | `volatile bool g_feedbackReceived=false;` | 表示是否至少收到过目标电机反馈；避免从未收到时把初值零当成有效在线时间。 |
| 34-35 | `float g_posHomeDeg=0.0f;` | 保存位置正弦的中心位置，单位度。真正的值稍后由反馈锁存。 |
| 36 | `bool g_homeLatched=false;` | 标记 home 是否已锁存，避免每拍改变摆动中心。 |
| 38-39 | `void FeedWatchdog(void)` | 定义不带参数、不返回值的喂狗辅助函数。参数里的 void 表示无参数。 |
| 40 | `#if WATCHDOG_ENABLE` | 编译期条件，宏为 1 时才保留下方刷新语句。不是关闭 IWDG 硬件的开关。 |
| 41 | `HAL_IWDG_Refresh(&hiwdg);` | 通过句柄地址刷新独立看门狗计数器，推迟超时复位。 |
| 42 | `#endif` | 结束编译期条件。 |
| 45-46 | `void SendCurrentCmd(void)` | 将当前保存的命令打包并提交给 CAN1。不是在此处计算 PID。 |
| 47 | `CAN_TxHeaderTypeDef tx_header;` | 定义 CAN 发送头结构体局部变量，随后给当前标准帧所需字段赋值。 |
| 48 | `uint8_t tx_data[8]={0};` | 定义并清零 8 字节数组，给非本电机的槽位发零命令。 |
| 49 | `uint32_t mailbox;` | 用来接收 HAL 选择了哪个发送邮箱，值通过指针由 HAL 写回。 |
| 51 | `g_motor.encode(tx_data);` | 数组作为地址传入，电机对象把本电机两个字节写进去。 |
| 53 | `tx_header.StdId=g_motor.txId();` | 设置标准 ID，ID=2 时用 0x1FE。 |
| 54 | `tx_header.IDE=CAN_ID_STD;` | 选择 11 位标准 ID 类型。不是设定数据长度。 |
| 55 | `tx_header.RTR=CAN_RTR_DATA;` | 选择数据帧，不是远程请求帧。 |
| 56 | `tx_header.DLC=8;` | 数据长度设为 8 字节，不含 ID、CRC 等开销。 |
| 57 | `tx_header.TransmitGlobalTime=DISABLE;` | 不启用 CAN 时间触发相关的全局时间发送功能。与本程序的 g_tick 不是同一个机制。 |
| 59 | `(void)HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&mailbox);` | 把报文提交给一个空闲发送邮箱并请求发送；使用 & 传地址。这里明确丢弃返回值，所以失败未处理；提交成功也不等于电机已经收到并执行。 |
| 62 | `bool FeedbackIsFresh(void)` | 返回反馈是否足够新，供控制前的保护判断使用。 |
| 63 | `if (!g_feedbackReceived) return false;` | ! 表示取反；尚未收到首帧就直接判为无有效反馈。 |
| 64 | `return (g_tick-g_lastFeedbackTick)<=CAN_FEEDBACK_TIMEOUT_TICKS;` | 用无符号差值比较控制节拍间隔。当前差值 100 仍通过，101 不通过。 |
| 67 | `} // namespace` | 匿名命名空间结束，后面的 C 链接函数对外使用。 |
| 69 | `extern "C" void ControlTaskInit(void)` | 定义可被 C 主程序链接调用的初始化接口；extern "C" 不会把函数体改成 C 语法。 |
| 70-71 | `CAN_FilterTypeDef filter={0};` | 定义过滤器配置并将成员清零，随后逐项填入；未显式赋值的成员仍为零。 |
| 72 | `filter.FilterBank=0;` | 使用过滤器组 bank 0。CAN1/2 的归属还取决于分界配置。 |
| 73 | `filter.FilterMode=CAN_FILTERMODE_IDMASK;` | 使用 ID/掩码匹配模式。 |
| 74 | `filter.FilterScale=CAN_FILTERSCALE_32BIT;` | 选择 32 位过滤器尺度；不表示报文 ID 有 32 位。 |
| 75 | `filter.FilterIdHigh=0x0000;` | 设置匹配字段的高半字为零。 |
| 76 | `filter.FilterIdLow=0x0000;` | 设置匹配字段的低半字为零。 |
| 77 | `filter.FilterMaskIdHigh=0x0000;` | 高半字掩码全零，不要求这些位相同。 |
| 78 | `filter.FilterMaskIdLow=0x0000;` | 低半字掩码全零，IDE/RTR 也未限制。注释“所有标准帧”不严谨，实际意图是全放行。 |
| 79 | `filter.FilterFIFOAssignment=CAN_RX_FIFO0;` | 通过过滤器的帧进入接收 FIFO0。 |
| 80 | `filter.FilterActivation=ENABLE;` | 启用这个过滤器组。 |
| 81 | `HAL_CAN_ConfigFilter(&hcan1,&filter);` | 将配置写入 CAN 过滤器硬件；当前没检查失败，也漏设 SlaveStartFilterBank，见讲义代码现状。 |
| 83-84 | `HAL_CAN_Start(&hcan1);` | 启动 CAN1 工作；不是只靠初始化函数就自动开始接收。 |
| 85 | `HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);` | 允许 FIFO0 有待处理帧时产生通知/中断；NVIC 也要已使能。 |
| 87 | `g_tick=0;` | 清控制节拍计数。 |
| 88 | `g_lastFeedbackTick=0;` | 清最近反馈计数。 |
| 89 | `g_feedbackReceived=false;` | 设置为尚无反馈。该重置放在开中断后有首帧被覆盖的时序风险。 |
| 90 | `g_speedPid.reset();` | 清速度环历史。 |
| 91 | `g_posPid.reset();` | 清位置环历史。 |
| 93 | 关于主循环卡死的注释 | 不准确：定时器中断仍运行时仍会喂狗。必须按真实调用位置解释。 |
| 94 | `FeedWatchdog();` | 初始化结束附近先刷新一次，给后续启动留出时间。 |
| 95 | `HAL_TIM_Base_Start_IT(&htim6);` | 启动 TIM6 基本定时器并开启更新中断，后续周期进入控制。 |
| 98 | `extern "C" void CanFeedbackCallback(uint32_t std_id,const uint8_t *data)` | 定义接收后的应用层分发接口；调用者传入报文 ID 和数据地址。 |
| 99 | `if (std_id==g_motor.rxId())` | 只处理目标电机反馈 ID，当前为 0x206。这里只比 ID，不验证数据长度和帧类型。 |
| 100 | `g_motor.decode(data);` | 解析并更新电机对象里的角度、转速、电流、温度和跨圈状态。 |
| 101 | `g_lastFeedbackTick=g_tick;` | 记录本帧到达时的控制节拍，供超时判断。不是电机报文内自带的时间。 |
| 102 | `g_feedbackReceived=true;` | 标记至少收到一帧目标反馈。 |
| 106 | `extern "C" void MainTask(void)` | 定义每 1ms 进入的控制函数，运行于 TIM6 中断上下文。 |
| 107 | `FeedWatchdog();` | 每拍一开始刷新 IWDG；尚未检查 CAN 或计算是否正常，因此只能证明执行到了这里。 |
| 109 | `g_tick++;` | 控制计数加 1。固定周期下可据此估算经过的时间。 |
| 110 | `float t=static_cast<float>(g_tick)*CONTROL_PERIOD_S;` | 计数转 float 再乘 0.001，得到正弦相位使用的秒数。 |
| 112-113 | `if (!FeedbackIsFresh())` | 反馈未到或超过阈值，进入停输出分支。 |
| 114 | `g_speedPid.reset();` | 清速度环旧积分和误差。 |
| 115 | `g_posPid.reset();` | 清位置环旧历史。不会自动清 home 和累计圈数。 |
| 116 | `g_motor.setCurrent(0);` | 将软件命令设置为零电流。 |
| 117 | `SendCurrentCmd();` | 尝试发送零命令；总线故障时不保证到达，当前也未处理发包错误。 |
| 118 | `return;` | 提前结束本拍，跳过后面的跟随和第二次发送。 |
| 121 | `if (CONTROL_MODE==0)` | 宏为 0 时进入速度分支；它是普通 if，但条件来自编译时宏，改宏需要重新编译。任何非 0 值都会走 else，当前没有模式合法性检查。 |
| 122-123 | `float ref_vel_rpm=SPEED_SINE_AMP_RPM*std::sinf(kTwoPi*SPEED_SINE_FREQ_HZ*t);` | 计算目标转速。振幅 100rpm，频率 0.25Hz，输入相位为弧度。 |
| 124 | `float out=g_speedPid.calc(ref_vel_rpm,g_motor.velRpm());` | 目标与反馈都为 rpm，速度 PI 返回电流给定码值。 |
| 125 | `g_motor.setCurrent(static_cast<int16_t>(out));` | 将限幅后的 float 输出截去小数转为 int16，再保存并做协议限幅。不是此处立即发 CAN。 |
| 126 | `} else {` | 进入位置跟随分支，两种模式这一次只执行其中一个。 |
| 127-129 | `if (!g_homeLatched)` | 第一次有效进入位置控制时锁定中心，以后保持中心不变。 |
| 130 | `g_posHomeDeg=g_motor.angle()*kRad2Deg;` | 获取累计 rad 角度，乘 180/π 转成度作为 home。 |
| 131 | `g_homeLatched=true;` | 记录已锁存，后续不重复锁存。 |
| 134-135 | `float ref_pos_deg=g_posHomeDeg+POS_SINE_AMP_DEG*std::sinf(kTwoPi*POS_SINE_FREQ_HZ*t);` | 计算围绕 home 的位置正弦目标，单位度，默认振幅 180°。 |
| 136 | `float fdb_pos_deg=g_motor.angle()*kRad2Deg;` | 当前反馈位置转为度，与 ref_pos_deg 单位相同。这里读取与上面的 home 读取不是一份显式一致性快照。 |
| 138-139 | `float ref_vel_rpm=g_posPid.calc(ref_pos_deg,fdb_pos_deg);` | 外环把位置误差变为目标 rpm，限幅 ±300。当前仅 P 起作用，8rpm/度。 |
| 141 | 内环输出控制电压的注释 | 过时，实际下方调用 setCurrent，所以现在是电流给定。 |
| 142 | `float out=g_speedPid.calc(ref_vel_rpm,g_motor.velRpm());` | 内环根据外环给的目标转速与反馈转速算电流命令。 |
| 143 | `g_motor.setCurrent(static_cast<int16_t>(out));` | float 转 int16，保存电流命令并限幅。 |
| 146 | `SendCurrentCmd();` | 任一正常跟随分支执行完后统一打包发送。 |
| 149-152 | `extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)` | 用同名强定义覆盖 HAL 默认弱回调，由 HAL 的定时器中断处理调用。 |
| 153 | `if (htim->Instance==TIM6)` | 检查本次回调来自哪个定时器，避免其他定时器到时也做电机控制。 |
| 154 | `MainTask();` | 执行一拍控制；不是另起线程，而是在当前中断调用链中运行。 |
| 158 | `extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)` | 覆盖 FIFO0 待接收报文的 HAL 弱回调。句柄指出哪个 CAN 外设触发。 |
| 159 | `CAN_RxHeaderTypeDef rx_header;` | 准备存放接收帧头：ID、IDE、RTR、DLC 等。 |
| 160 | `uint8_t rx_data[8];` | 准备接收 8 字节缓冲，当前没有零初始化，因此不能在未验证短帧时假定未写入部分有有效内容。 |
| 162 | `if (hcan!=&hcan1 \|\| HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data)!=HAL_OK)` | 逻辑或具有短路性：不是 CAN1 就不取帧；否则取一帧，失败也进入分支。&hcan1 用于比较句柄地址；&rx_header 传帧头地址，rx_data 数组在这里转换为数据首地址。 |
| 163 | `return;` | 忽略非 CAN1 回调或本次取帧失败。 |
| 165 | `CanFeedbackCallback(rx_header.StdId,rx_data);` | 将帧 ID 和数据交给应用解析。目前未先检查标准数据帧及 DLC=8，是待补的验证步骤。 |

追问：为什么发送函数的 `tx_data` 是局部数组还可以？本地 HAL 实现在返回前把数据复制到硬件邮箱寄存器，未保存这个局部数组指针供以后使用；不能把这一点推广到所有异步 API。

追问：同样写 `const`，为什么 `decode(const uint8_t*)` 和 `encode(uint8_t*) const` 不一样？前者约束经指针对字节的修改，后者约束成员函数对电机对象普通成员的修改。

## ControlTask.h

| 行 | 语句或字段 | 怎样解释 |
| --- | --- | --- |
| 1-2 | `#ifndef/#define CONTROLTASK_H` | 头文件保护。 |
| 4 | `#include <stdint.h>` | 采用 C/C++ 都能使用的固定位宽整数头文件，以声明 uint32_t、uint8_t 接口参数。 |
| 6-7 | `#ifdef __cplusplus` | 只有 C++ 编译器定义了这个宏时才展开 C++ 特有的链接语法。 |
| 8 | `extern "C" {` | 其中的函数声明使用 C 链接命名，方便 main.c 调用以及 HAL 回调匹配。 |
| 9 | `#endif` | 结束预处理条件，不是关闭上面的链接语句块。 |
| 11-13 | `void ControlTaskInit(void);` | 声明初始化入口，在外设初始化之后调用一次。 |
| 15-16 | `void MainTask(void);` | 声明一拍控制入口，当前由 TIM6 回调调用。 |
| 18-19 | `void CanFeedbackCallback(uint32_t std_id,const uint8_t *data);` | 声明应用层反馈分发入口，含报文 ID 和只读字节地址。 |
| 21 | `#ifdef __cplusplus` | C++ 编译时才需要关闭前面打开的 C 链接块。 |
| 22 | `}` | 关闭 extern "C" 块，不会在运行时执行。 |
| 23 | `#endif` | 结束 C++ 条件。 |
| 25 | `#endif` | 结束整个头文件保护。 |

## config.h

注释说明设计意图，宏提供编译时数值。参数应和单位一起记，不应只背数字。

| 行 | 宏或注释 | 怎样解释 |
| --- | --- | --- |
| 1-2 | `#ifndef/#define CONFIG_H` | 配置头文件保护。 |
| 4-14 | 看门狗说明 | 初始化在 Core 中已经执行。约 0.8s 是估算；宏设 0 仅停止喂狗，不是关闭硬件保护。 |
| 15 | `WATCHDOG_ENABLE 1` | 默认编译喂狗语句。 |
| 17-20 | `CONTROL_FREQ_HZ 1000.0f` | 设计控制频率 1000Hz，应与 TIM6 实际配置一致；改它不会自动修改 TIM6。 |
| 21 | `CONTROL_PERIOD_S (1.0f / CONTROL_FREQ_HZ)` | 设计控制周期 0.001s，轨迹时间换算使用。 |
| 23-26 | `MOTOR_ID 2` | 目标电机拨码 ID，决定接收 ID 和发送组、槽位。 |
| 28-29 | `CAN_FEEDBACK_TIMEOUT_MS 100` | 反馈超时阈值 100ms。 |
| 30 | `#define CAN_FEEDBACK_TIMEOUT_TICKS \` | 定义多行宏，行末反斜杠表示宏内容在下一行继续。 |
| 31 | `((uint32_t)(CAN_FEEDBACK_TIMEOUT_MS * CONTROL_FREQ_HZ / 1000.0f))` | 毫秒换成控制节拍，当前结果 100；这里是 C 风格类型转换，转换为 uint32_t。宏依赖已经包含整数类型定义。 |
| 33-38 | `CONTROL_MODE 0` | 0 为速度曲线；1 为位置曲线。当前按非零进入 else，没有自动循环切换模式。 |
| 40-44 | `SPEED_SINE_AMP_RPM 100.0f` | 速度正弦振幅 100rpm，目标范围 -100..100。 |
| 45 | `SPEED_SINE_FREQ_HZ 0.25f` | 正弦频率 0.25Hz，周期 4s。不是控制频率。 |
| 47-51 | `POS_SINE_AMP_DEG 180.0f` | 位置正弦振幅 180°，围绕锁存 home。 |
| 52 | `POS_SINE_FREQ_HZ 0.25f` | 位置轨迹频率 0.25Hz，也是 4s 一周期。 |
| 54-57 | `SPEED_PID_KP 15.0f` | 速度误差每增加 1rpm，P 输出增加 15 个电流给定码值。 |
| 58 | `SPEED_PID_KI 1.0f` | 每一拍按当前误差累加积分输出，周期已包含在参数定义中。 |
| 59 | `SPEED_PID_KD 0.0f` | 不启用速度微分项。 |
| 60 | `SPEED_PID_I_MAX 140.0f` | 速度环积分输出限制在 -140..140 码值。 |
| 61 | `SPEED_PID_OUT_MAX 16384.0f` | 速度环总输出正向上限，电流命令满量程。不是目标速度上限。 |
| 62 | `SPEED_PID_OUT_MIN (-16384.0f)` | 速度环总输出负向下限，括号避免宏展开时与其他运算结合出意外。 |
| 64-67 | `POS_PID_KP 8.0f` | 每 1° 位置误差，位置 P 产生 8rpm 的速度给定，之后还要限幅。 |
| 68 | `POS_PID_KI 0.0f` | 位置环不积分。 |
| 69 | `POS_PID_KD 0.0f` | 位置环不做微分。 |
| 70 | `POS_PID_I_MAX 100.0f` | 位置环积分项上限，当前 Ki=0 且历史初始为零，积分项一直为零。 |
| 71 | `POS_PID_OUT_MAX 300.0f` | 位置环输出转速的正向上限，rpm。 |
| 72 | `POS_PID_OUT_MIN (-300.0f)` | 位置环输出转速的负向下限，rpm。 |
| 74 | `#endif` | 配置头文件保护结束。 |

## 与 Core 的连接：还要知道这几处

你限定可改的是 Tasks，但考官可能追问“是谁调用了它”。这些是只读理解点。

| 位置 | 作用 | 应答要点 |
| --- | --- | --- |
| `Core/Src/main.c` 的 `HAL_Init()` | 初始化 HAL 基础设施和系统节拍等 | 先有基础运行环境，再初始化具体外设 |
| `SystemClock_Config()` | 配置系统和总线时钟，启用 LSI 等 | CAN、TIM 和 IWDG 的时钟来源并不都相同 |
| `MX_CAN1_Init()` | 配置 CAN1 | 42MHz / [3×(1+6+7)] = 1Mbps |
| `MX_TIM6_Init()` | 配置 TIM6 | 84MHz / [(83+1)×(999+1)] = 1kHz |
| `MX_IWDG_Init()` | 启动并配置独立看门狗 | 64 分频、Reload=400，标称约 0.802s |
| `ControlTaskInit()` | 把 Tasks 逻辑接入外设 | 启动接收通知和控制中断 |
| `while(1)` | 主循环当前为空 | 中断仍可运行，不能说空循环表示没做控制 |
| `CAN1_RX0_IRQHandler` | 接收 IRQ 入口 | 调 HAL_CAN_IRQHandler，再调到 Tasks 的回调 |
| `TIM6_DAC_IRQHandler` | 定时器 IRQ 入口 | 调 HAL_TIM_IRQHandler，再调控制回调 |
| CAN1 PD0/PD1 配置 | MCU CAN RX/TX 的复用功能引脚 | 它们通过收发器转成总线 H/L，不等于直接连接电机的 H/L |
| `HAL_NVIC_SetPriority(...,1,0)` 和 `(...,2,0)` | CAN 接收与 TIM6 的中断优先级 | 当前 CAN 可抢占 TIM6，需考虑共享数据 |

## 一行代码的合格回答示范

考官指着 `float out = g_speedPid.calc(ref_vel_rpm, g_motor.velRpm());`：

“这一行调用速度控制器，目标转速和电机反馈转速都用 rpm。内部算目标减反馈的误差，再算比例项和积分项并限幅；当前微分增益是零，所以实际上是 PI。返回的是电流给定码值，保存在浮点局部变量 out 中。下一行会转成有符号 16 位并保存到电机对象，后面才统一打包发送 CAN。这一行自己不会发 CAN，也不会直接让速度立即等于目标。”

考官指着 `data[index] = static_cast<uint8_t>((voltage_ >> 8) & 0xFF);`：

“它把当前命令的高字节写到电机所属的数据槽位。voltage_ 名字旧了，当前是电流给定。每台电机两字节，高字节在前；右移 8 位再保留低 8 位取出原命令的高字节。ID=2 时 index=2，给定 1000 是 0x03E8，这一行写入 0x03，下一行写 0xE8。”

考官指着 `HAL_IWDG_Refresh(&hiwdg);`：

“将看门狗句柄地址传给 HAL 刷新函数，把计数器重新装载，避免在本次超时窗口内复位。IWDG 已在 main 的外设初始化阶段启动，标称超时约 0.8 秒。当前这行从 TIM6 控制中断执行，说明定时器中断到了这里，不能证明主循环或 CAN 全部正常。”
