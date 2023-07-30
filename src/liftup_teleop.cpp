#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "utils/first_order_filter.hpp"
#include "gary_msgs/srv/reset_motor_position.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace gary_liftup {

    class LiftupTeleop : public rclcpp::Node {
    public:
        LiftupTeleop(const rclcpp::NodeOptions &options) : rclcpp::Node("liftup_teleop", options) {
            this->cmd_sub = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",
                                                                                    rclcpp::SystemDefaultsQoS(),
                                                                                    std::bind(
                                                                                            &LiftupTeleop::cmd_callback,
                                                                                            this,
                                                                                            std::placeholders::_1));
            this->twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
                                                                                      rclcpp::SystemDefaultsQoS());
            this->rescue_left_publisher = this->create_publisher<std_msgs::msg::Float64>("/rescue_left_pid/cmd",
                                                                                         rclcpp::SystemDefaultsQoS());
            this->rescue_right_publisher = this->create_publisher<std_msgs::msg::Float64>("/rescue_right_pid/cmd",
                                                                                          rclcpp::SystemDefaultsQoS());
            this->liftup_left_publisher = this->create_publisher<std_msgs::msg::Float64>("/liftup_left_pid/cmd",
                                                                                         rclcpp::SystemDefaultsQoS());
            this->liftup_right_publisher = this->create_publisher<std_msgs::msg::Float64>("/liftup_right_pid/cmd",
                                                                                          rclcpp::SystemDefaultsQoS());
            this->stretch_left_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_left_pid/cmd",
                                                                                          rclcpp::SystemDefaultsQoS());
            this->stretch_right_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_right_pid/cmd",
                                                                                           rclcpp::SystemDefaultsQoS());
            this->pitch_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_pid/cmd",
                                                                                   rclcpp::SystemDefaultsQoS());
            this->yaw_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_pid/cmd",
                                                                                 rclcpp::SystemDefaultsQoS());
            this->roll_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_roll_pid/cmd",
                                                                                  rclcpp::SystemDefaultsQoS());
            this->arm_publisher = this->create_publisher<std_msgs::msg::Float64>("/gimbal_arm_pid/cmd",
                                                                                  rclcpp::SystemDefaultsQoS());

            this->reset_position_client = this->create_client<gary_msgs::srv::ResetMotorPosition>("/reset_motor_position");

            this->timer = this->create_wall_timer(10ms, [this] { timer_callback(); });

            this->x_filter = new First_orderFilter(1.5);
            this->y_filter = new First_orderFilter(1.5);
        }

        void cmd_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
            this->rc = *msg;
        }

        void timer_callback() {
            static double liftup_set = 0;
            static double rescue_set = 0;
            static double stretch_set = 0;
            static double pitch_set = 0;
            static double yaw_set = 1.05;
            static double roll_set = 0;
            static double arm_set = 0;
            static bool use_sucker = false;
            static bool calibration_mode = false;
            static int calibration_count = 0;
            double vx_set = 0;
            double vy_set = 0;
            double az_set = 0;
            bool use_speed_filter = true;

            static std::chrono::time_point<std::chrono::steady_clock> last_ctrl;

            if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_MID) {
                if (this->rc.sw_left == gary_msgs::msg::DR16Receiver::SW_DOWN) {
                    vx_set = this->rc.ch_left_y * 4.0f;
                    vy_set = - this->rc.ch_right_x * 4.0f;
                    az_set = - this->rc.ch_wheel * 4.0f;
                } else if (this->rc.sw_left == gary_msgs::msg::DR16Receiver::SW_MID) {
                    liftup_set += this->rc.ch_left_y * 0.05;
                    stretch_set += this->rc.ch_right_y * 0.15;
                    rescue_set += this->rc.ch_wheel * 0.1;
                } else if (this->rc.sw_left == gary_msgs::msg::DR16Receiver::SW_UP) {
                    pitch_set -= this->rc.ch_left_y * 0.02;
                    yaw_set -= this->rc.ch_right_x * 0.01;
                    roll_set -= this->rc.ch_wheel * 0.1;
                }
            } else if (this->rc.sw_right == gary_msgs::msg::DR16Receiver::SW_UP) {

                //chassis movement
                if (this->rc.key_shift || liftup_set > 4) {
                    if (this->rc.key_w) {
                        vx_set += 0.4f;
                    } else if (this->rc.key_s) {
                        vx_set += - 0.4f;
                    }
                    if (this->rc.key_a) {
                        vy_set += 0.4f;
                    } else if (this->rc.key_d) {
                        vy_set += - 0.4f;
                    }
                } else {
                    if (this->rc.key_w) {
                        vx_set += 4.0f;
                    } else if (this->rc.key_s) {
                        vx_set += - 4.0f;
                    }
                    if (this->rc.key_a) {
                        vy_set += 4.0f;
                    } else if (this->rc.key_d) {
                        vy_set += - 4.0f;
                    }
                }

                if (! this->rc.key_ctrl) {
                    az_set += this->rc.mouse_x * 0.02 * log2(fmax(fabs(vx_set),fabs(vy_set)) + 2);
                }

                if (this->rc.key_x) {
                    if(!this->rc.key_ctrl) {
                        az_set -= 2.0;
                    }
                } else if (this->rc.key_c) {
                    if(!this->rc.key_ctrl) {
                        az_set += 2.0;
                    }
                }

                if (calibration_mode) {
                    static bool reset_position = false;
                    calibration_count++;

                    if (calibration_count < 300) {
                        rescue_set = -2.2;
                        stretch_set = -28.0;
                        pitch_set = 3.85;
//                        arm_set = 2.10;

                        std_msgs::msg::Float64 send;
                        send.data = rescue_set;
                        this->rescue_left_publisher->publish(send);
                        send.data = -rescue_set;
                        this->rescue_right_publisher->publish(send);

                        send.data = stretch_set;
                        this->stretch_left_publisher->publish(send);
                        send.data = -stretch_set;
                        this->stretch_right_publisher->publish(send);

                        send.data = pitch_set;
                        this->pitch_publisher->publish(send);

//                        send.data = arm_set;
//                        this->arm_publisher->publish(send);
                    } else if (calibration_count > 300 && calibration_count < 400) {
                        if (! reset_position) {
                            if (this->reset_position_client->service_is_ready()) {
                                auto req = std::make_shared<gary_msgs::srv::ResetMotorPosition::Request>();
                                req->motor_name = "rescue_left";
                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "rescue_right";
                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "liftup_left";
                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "liftup_right";
                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "gimbal_pitch";
                                this->reset_position_client->async_send_request(req);
//                                req->motor_name = "gimbal_arm";
//                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "gimbal_left";
                                this->reset_position_client->async_send_request(req);
                                req->motor_name = "gimbal_right";
                                this->reset_position_client->async_send_request(req);
                                reset_position = true;
                            } else {
                                this->reset_position_client->wait_for_service(10ms);
                            }
                        }
                    } else if (calibration_count > 400) {
                        calibration_count = 0;
                        calibration_mode = false;
                        reset_position = false;
                        RCLCPP_INFO(this->get_logger(), "exiting calibration mode");
                    }
                    return;
                }

                vx_set += this->rc.ch_left_y * 0.8f;
                vy_set += - this->rc.ch_left_x * 0.8f;
                az_set +=  this->rc.ch_right_x * 0.8f;
                liftup_set += this->rc.ch_right_y * 0.05;

                //sucker
                if (this->rc.ch_wheel > 0.5) {
                    use_sucker = false;
                } else if (this->rc.ch_wheel < -0.5){
                    use_sucker = true;
                }
                if (this->rc.mouse_press_l && ! this->rc.mouse_press_r) {
                    use_sucker = true;
                } else if (this->rc.mouse_press_l && this->rc.mouse_press_r) {
                    use_sucker = false;
                }

                //liftup
                if (this->rc.mouse_press_r) {
                    if (this->rc.key_f) {
                        liftup_set = 14.0f;
                    } else if (this->rc.key_v) {
                        liftup_set = 2.9f;
                    }
                } else {
                    if (this->rc.key_f) {
                        liftup_set += 0.05f;
                    } else if (this->rc.key_v) {
                        liftup_set -= 0.05f;
                    }
                }

                //stretch
                if ( ! this->rc.mouse_press_r) {
                    if (this->rc.key_q) {
                        stretch_set +=  0.15;
                    } else if(this->rc.key_e){
                        stretch_set -=  0.15;
                    }
                }

                //gimbal
                pitch_set -= this->rc.mouse_y * 0.0009;

                if (this->rc.key_ctrl) {
                    yaw_set -= this->rc.mouse_x * 0.0003;
                }

                if (this->rc.mouse_press_r) {
                    if (this->rc.key_q) {
                        roll_set -= 0.025f;
                    } else if (this->rc.key_e) {
                        roll_set += 0.025f;
                    }
                }

                //gimbal reset
                if (this->rc.key_r) {
                    yaw_set = 1.05;
                    arm_set = -2.0;
                    pitch_set = -1.5;
                    stretch_set = 0.0;
                }

                //rescue
                if (this->rc.key_z && this->rc.key_ctrl) {
                    rescue_set = 2.2f;
                } else if (this->rc.key_z) {
                    rescue_set = 0.0f;
                }

                //arm && one-key script
                {
                    static bool one_key_exec = false;
                    static bool storing = false;
                    static bool getting = false;
                    static bool golding = false;
                    static bool set_once_flag = false;
                    static double last_roll = 0.0;
                    static std::chrono::time_point<std::chrono::steady_clock> action_exec_point;
                    static int key_press_count_ms = 0;
                    if(!one_key_exec) {
                        if(!getting && !storing){
                            if(!set_once_flag) {
                                pitch_set = -1.5;
                                arm_set = -2.0;
                                set_once_flag = true;
                            }
                        }
                        if (rc.key_c) {
                            if (rc.key_ctrl) {
                                key_press_count_ms += 10;
                                last_ctrl = std::chrono::steady_clock::now();
                            }
                            if (std::chrono::steady_clock::now() - last_ctrl <= 20ms && key_press_count_ms >= 600) {
                                one_key_exec = true;
                                storing = true;
                                action_exec_point = std::chrono::steady_clock::now();
                                last_roll = roll_set;
                                key_press_count_ms = 0;
                                RCLCPP_INFO(this->get_logger(),"Storing...");
                            }
                        }
                        if (rc.key_x) {
                            if (rc.key_ctrl) {
                                key_press_count_ms += 10;
                                last_ctrl = std::chrono::steady_clock::now();
                            }
                            if (std::chrono::steady_clock::now() - last_ctrl <= 20ms && key_press_count_ms >= 600) {
                                one_key_exec = true;
                                getting = true;
                                action_exec_point = std::chrono::steady_clock::now();
                                last_roll = roll_set;
                                key_press_count_ms = 0;
                                RCLCPP_INFO(this->get_logger(),"Getting...");
                            }
                        }
                        if (this->rc.key_b) {
                            one_key_exec = true;
                            golding = true;
                            action_exec_point = std::chrono::steady_clock::now();
                            RCLCPP_INFO(this->get_logger(),"Golding...");
                        }
                    } else {
                        if(key_press_count_ms > 0){
                            key_press_count_ms -= 10;
                        }
                        if(storing && !getting){
                            if(std::chrono::steady_clock::now() - action_exec_point <= 1s){
                                liftup_set = 14.0f;
                                stretch_set = 16.0;
                                pitch_set = 0.0;
                                yaw_set = 1.05;
                                arm_set = -2.0;
                                roll_set = last_roll + 6.28;
                                use_sucker = true;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 1.7s){
                                yaw_set = -2.13;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 2.2s){
                                stretch_set = 0.0;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 3.0s){
                                stretch_set = 0.0;
                                pitch_set = -0.9;
                                arm_set = -1.4;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 3.5s) {
                                use_sucker = false;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 3.7s){
                                stretch_set = 16.0;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 4.2s){
                                yaw_set = 1.05;
                            }else{
                                stretch_set = 0.0;
                                yaw_set = 1.05;
                                arm_set = -2.0;
                                pitch_set = -1.5;
                                liftup_set = 2.8f;
                                one_key_exec = false;
                                storing = false;
                                RCLCPP_INFO(this->get_logger(),"Stored.");
                            }
                        }else if(getting && !storing){
                            if(std::chrono::steady_clock::now() - action_exec_point <= 1s){
                                liftup_set = 14.0f;
                                stretch_set = 16.0;
                                pitch_set = -0.97;
                                arm_set = -1.25;
                                yaw_set = -2.13;
                                use_sucker = true;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 2.5s){
                                stretch_set = 0.0;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 2.9s){
                                stretch_set = 16.0;
                                pitch_set = 0.0;
                                arm_set = -2.0;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 3.5s){
                                yaw_set = 1.05;
                            }else if(std::chrono::steady_clock::now() - action_exec_point <= 3.9s){
                                roll_set = last_roll - 6.28;
                            }else{
                                stretch_set = 0.0;
                                yaw_set = 1.05;
                                arm_set = -2.0;
                                pitch_set = -1.5;
                                one_key_exec = false;
                                getting = false;
                                RCLCPP_INFO(this->get_logger(),"Got.");
                            }
                        }else if(golding){
                            if(std::chrono::steady_clock::now() - action_exec_point <= 1s){
                                yaw_set = 1.05;
                                arm_set = 0.0;
                                pitch_set = 0.0;
                                liftup_set = 6.615;
                                use_sucker = true;
                            }else{
                                stretch_set = 23.0;
                                one_key_exec = false;
                                golding = false;
                                RCLCPP_INFO(this->get_logger(),"Gold.");
                            }
                        }
                    }
                }

                //calibration
                if (this->rc.mouse_press_r && this->rc.key_z) {
                    calibration_count++;
                    if (calibration_count > 200) {
                        calibration_count = 0;
                        calibration_mode = true;
                        RCLCPP_INFO(this->get_logger(), "entering calibration mode");
                    }
                }else{
                    if(calibration_count>0) { calibration_count--; }
                }
            } else {
                return;
            }


            geometry_msgs::msg::Twist twist;
            if (use_speed_filter) {
                if (vx_set == 0) this->x_filter->reset();
                if (vy_set == 0) this->y_filter->reset();
                vx_set = this->x_filter->first_order_filter(vx_set);
                vy_set = this->y_filter->first_order_filter(vy_set);
                twist.linear.x = vx_set;
                twist.linear.y = vy_set;
                twist.angular.z = az_set;
            } else {
                twist.linear.x = vx_set;
                twist.linear.y = vy_set;
                twist.angular.z = az_set;
            }
            this->twist_publisher->publish(twist);

            if (liftup_set > 16.0) liftup_set = 16.0;
            if (liftup_set < 0.0) liftup_set = 0.0;
            if (stretch_set > 28.0) stretch_set = 28.0;
            if (stretch_set < 0.0) stretch_set = 0.0;
            if (rescue_set > 2.2) rescue_set = 2.2;
            if (rescue_set < 0.0) rescue_set = 0.0;
            if (pitch_set > 0.0) pitch_set = 0.0;
            if (pitch_set < -3.14) pitch_set = -3.14;
            if (yaw_set > 2.62) yaw_set = 2.62;
            if (yaw_set < -2.13) yaw_set = -2.13;

            std_msgs::msg::Float64 send;
            send.data = liftup_set;
            this->liftup_left_publisher->publish(send);
            send.data = -liftup_set;
            this->liftup_right_publisher->publish(send);

            send.data = rescue_set;
            this->rescue_left_publisher->publish(send);
            send.data = -rescue_set;
            this->rescue_right_publisher->publish(send);

            send.data = stretch_set;
            this->stretch_left_publisher->publish(send);
            send.data = -stretch_set;
            this->stretch_right_publisher->publish(send);

            send.data = pitch_set;
            this->pitch_publisher->publish(send);
            send.data = yaw_set;
            this->yaw_publisher->publish(send);
            send.data = roll_set;
            this->roll_publisher->publish(send);
            send.data = arm_set;
            this->arm_publisher->publish(send);

            if (use_sucker) {
                system("cansend can-gimbal 112#ffffffffffffffff");
            } else {
                system("cansend can-gimbal 112#0000000000000000");
            }

        }

    private:
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr cmd_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rescue_left_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rescue_right_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr liftup_left_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr liftup_right_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stretch_left_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stretch_right_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_publisher;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_publisher;
        rclcpp::Client<gary_msgs::srv::ResetMotorPosition>::SharedPtr reset_position_client;
        rclcpp::TimerBase::SharedPtr timer;
        gary_msgs::msg::DR16Receiver rc;
        First_orderFilter *x_filter;
        First_orderFilter *y_filter;
    };
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_liftup::LiftupTeleop> liftup_teleop = std::make_shared<gary_liftup::LiftupTeleop>(rclcpp::NodeOptions());

    exe.add_node(liftup_teleop->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_liftup::LiftupTeleop)
