#include "utils/first_order_filter.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace gary_liftup;

static inline bool DoubleEqual(double a, double b)
{
    return std::abs(a - b) < std::numeric_limits<double>::epsilon();
}

First_orderFilter::First_orderFilter(double max_accel) {
    this->max_accel = max_accel;
    this->last_time = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
    this->out = 0;
}

double First_orderFilter::first_order_filter(double input) {
    if(DoubleEqual(input,0.0) && this->out <= 0.0){
        this->out = 0;
        this->last_time = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
        return this->out;
    }
    double time_now = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
    double delta_time = time_now - this->last_time;
    this->last_time = time_now;
    double delta_accel = this->max_accel * delta_time;
    double delta_input = input - this->out;
    if (delta_input > delta_accel) delta_input = delta_accel;
    if (delta_input < 0.0 - (2.0*delta_accel)) delta_input = 0.0 - (2.0*delta_accel);
    this->out += delta_input;
    if(this->out >= 1.9) this->out = 1.9;
    if(this->out <= -1.9) this->out = -1.9;
    return this->out;
}

void First_orderFilter::reset() {
    this->out = 0;
    this->last_time = rclcpp::Clock{RCL_SYSTEM_TIME}.now().seconds();
}
