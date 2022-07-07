#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <chrono>
#include <thread>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

const double PI = 3.14159265359;
const double tolerance = 0.5;
const double coeff = 1;

double sinc(double t){
    if (abs(t) < 0.002)
        return 1 - (t*t*((1.0/6) - (t*t/120)));
    else
        return std::sin(t)/t;
}

double mod2pi(double w){
    double res = w;
    while (res < 0)
    {
        res += 2*PI;
    }
    while (res >= 2*PI)
    {
        res -= 2*PI;
    }
    
    return res;
}

double rangeSymm(double w){
    double res = w;
    if(!(res > -PI && res < PI)){
        if(res < 0){
            res += 2*PI;
        } else {
            res -= 2*PI;
        }
    }
    return res;
}

class Dubins : public rclcpp::Node {
public:
    explicit Dubins() : rclcpp::Node("dubins"){
        k_ = 1.0 / (0.2); // before 0.4
        v_ = 0.7;
        w_ = 0.3;
        v_ang_ = w_ / k_;
        RCLCPP_INFO(this->get_logger(), "v_: %f", v_);
        RCLCPP_INFO(this->get_logger(), "w_: %f", w_);
        RCLCPP_INFO(this->get_logger(), "k_: %f", k_);
        RCLCPP_INFO(this->get_logger(), "v_ang_: %f", v_ang_);

        cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS());
        completion_ = create_publisher<std_msgs::msg::Float64>("completion", rclcpp::SystemDefaultsQoS());

        pos_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&Dubins::pos_update_, this, std::placeholders::_1)
        );

        goal_ = create_subscription<geometry_msgs::msg::Pose2D>(
            "goal",
            10,
            std::bind(&Dubins::goal_handler_, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr completion_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
    double k_, w_, v_, v_ang_;
    double lambda;
    double pos_[3];

    void pos_update_(const nav_msgs::msg::Odometry::SharedPtr msg){
        /**pos_[0] = msg->transforms.back().transform.translation.x;
        pos_[1] = msg->transforms.back().transform.translation.y;
        pos_[2] = mod2pi(2*atan2(msg->transforms.back().transform.rotation.z, msg->transforms.back().transform.rotation.w));*/
        pos_[0] = msg->pose.pose.position.x;
        pos_[1] = msg->pose.pose.position.y;
        pos_[2] = mod2pi(2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    }

    void goal_handler_(const geometry_msgs::msg::Pose2D::SharedPtr msg){
        int min; double s[3];
        float x1 = msg->x;  float y1 = msg->y;  float th1 = msg->theta;
        get_dubins_maneouvres(x1, y1, th1, &min , s);
        execute_maneouvres(min, s);

        std_msgs::msg::Float64 feedback;
        feedback.data = 1.0;
        completion_->publish(feedback);
    }

    void get_dubins_maneouvres(float x1, float y1, float th_1, int* min, double *s){
        double cost, min_cost = -1;
        double x0 = pos_[0], y0 = pos_[1],   th_0 = pos_[2];

        double gamma = atan2((y1 - y0), (x1 - x0));
        lambda = (sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))) / 2;
        double th0 = th_0 - gamma; double th1 = th_1 - gamma;
        double k = lambda*k_;

        bool res;   double tmp_s[3];

        //LSL
        res = LSL(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff + tmp_s[1] + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 0, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 0;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }

        //RSR
        res = RSR(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff+ tmp_s[1] + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 1, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 1;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }
        
        //LSR
        res = LSR(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff + tmp_s[1] + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 2, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 2;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }
        
        //RSL
        res = RSL(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff + tmp_s[1] + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 3, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 3;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }

        //RLR
        res = RLR(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff + tmp_s[1]*coeff + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 4, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 4;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }
        
        //LRL
        res = LRL(tmp_s, th0, th1, k);
        if(res){
            cost = tmp_s[0]*coeff + tmp_s[1]*coeff + tmp_s[2]*coeff;
            // RCLCPP_INFO(this->get_logger(), "sol: %d, cost: %f", 5, cost);
            if(cost < min_cost || min_cost == -1){
                min_cost = cost;
                *min = 5;
                s[0] = tmp_s[0]; s[1] = tmp_s[1]; s[2] = tmp_s[2];
            }
        }

        s[0] *= lambda; s[1] *= lambda; s[2] *= lambda;

    }

    void execute_maneouvres(int min, double *s){
        std_msgs::msg::Float64 comp;
        comp.data = 0;
        completion_->publish(comp);
        if(min == 0 || min == 2 || min == 5){
            //LEFT
            move_left(k_ * int(s[0] * 1000 / w_));
        } else {
            //RIGHT
            move_right(k_ * int(s[0] * 1000 / w_));
        }

        comp.data = 1./3;
        completion_->publish(comp);

        if(min == 4){
            //LEFT
            move_left(int(k_* s[1] * 1000 / w_));
        } else if(min == 5){
            //RIGHT
            move_right(int(k_*s[1] * 1000 / w_));
        } else {
            //STRAIGHT
            move_straight(int(s[1] * 1000 / v_));
        }

        comp.data = 0.6;
        completion_->publish(comp);

        if(min == 0 || min == 3 || min == 5){
            //LEFT
            move_left(int(k_*s[2] * 1000 / w_));
        } else {
            //RIGHT
            move_right(int(k_*s[2] * 1000 / w_));
        }

        comp.data = 0.90;
        completion_->publish(comp);
    }

    bool LSL(double* s, double th_0, double th_1, double k){
        double invK = 1 / k;
        double C = std::cos(th_1) - std::cos(th_0);
        double S = 2*k + std::sin(th_0) - std::sin(th_1);

        s[0] = invK * mod2pi(atan2(C, S) - th_0);
        double tmp = 2 + (4*k*k) - (2*std::cos(th_0 - th_1)) + (4*k*(std::sin(th_0) - std::sin(th_1)));
        if(tmp < 0){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false;
        }
        s[1] = invK * sqrt(tmp);
        s[2] = invK * mod2pi(th_1 - atan2(C, S));

        double _k[3];
        _k[0] = 1;  _k[1] = 0;   _k[2] = 1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool RSR(double* s, double th_0, double th_1, double k){
        double inv_k = 1 / k;
        double C = std::cos(th_0) - std::cos(th_1);
        double S = 2*k - std::sin(th_0) + std::sin(th_1);

        s[0] = inv_k * mod2pi(th_0 - atan2(C, S));
        double tmp = 2 + (4*k*k) - (2*std::cos(th_0 - th_1)) - (4*k*(std::sin(th_0) - std::sin(th_1)));
        if(tmp < 0){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false;
        }
        s[1] = inv_k * sqrt(tmp);
        s[2] = inv_k * mod2pi(atan2(C, S) - th_1);
        
        double _k[3];
        _k[0] = -1;  _k[1] = 0;   _k[2] = -1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool LSR(double* s, double th_0, double th_1, double k){
        double inv_k = 1 / k;
        double C = std::cos(th_0) + std::cos(th_1);
        double S = (2*k) + std::sin(th_0) + std::sin(th_1);
        double tmp = (4*k*k) - 2 + (2*std::cos(th_0-th_1)) + (4*k*(std::sin(th_0) + std::sin(th_1)));

        if(tmp < 0){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false;
        }
        s[1] = inv_k * sqrt(tmp);
        s[0] = inv_k * mod2pi(atan2(-C, S) + (-atan2(-2, k*s[1])) - th_0);
        s[2] = inv_k * mod2pi(atan2(-C, S) + (-atan2(-2, k*s[1])) - th_1);

        double _k[3];
        _k[0] = 1;  _k[1] = 0;   _k[2] = -1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool RSL(double* s, double th_0, double th_1, double k){
        double inv_k = 1 / k;
        double C = std::cos(th_0) + std::cos(th_1);
        double S = (2*k) - std::sin(th_0) - std::sin(th_1);
        double tmp = (4*k*k) - 2 + (2* std::cos(th_0 -  th_1)) - (4*k*(std::sin(th_0) + std::sin(th_1)));
       
        if(tmp < 0){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false;
        }
        s[1] = inv_k * sqrt(tmp);
        s[0] = inv_k * mod2pi(th_0 - atan2(C, S) + atan2(2, k*s[1]));
        s[2] = inv_k * mod2pi(th_1 - atan2(C, S) + atan2(2, k*s[1]));
        
        double _k[3];
        _k[0] = -1;  _k[1] = 0;   _k[2] = 1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool RLR(double* s, double th_0, double th_1, double k){
        double inv_k = 1 / k;
        double C = std::cos(th_0) - std::cos(th_1);
        double S = (2*k) + std::sin(th_0) - std::sin(th_1);
        double tmp = (6 - (4*k*k) + (2* std::cos(th_0 -  th_1)) + (4*k*(std::sin(th_0) - std::sin(th_1)))) / 8;
        
        if(abs(tmp) > 1){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false;
        }
        s[1] = inv_k * mod2pi(2*PI - acos(tmp));
        s[0] = inv_k * mod2pi(th_0 - atan2(C, S) + k*s[1]/2);
        s[2] = inv_k * mod2pi(th_0 - th_1 + k*(s[1] - s[0]));

        double _k[3];
        _k[0] = -1;  _k[1] = 1;   _k[2] = -1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool LRL(double* s, double th_0, double th_1, double k){
        double inv_k = 1 / k;
        double C = std::cos(th_0) - std::cos(th_1);
        double S = (2*k) + std::sin(th_0) - std::sin(th_1);
        double tmp = (6 - (4*k*k) + (2* std::cos(th_0 -  th_1)) - (4*k*(std::sin(th_0) - std::sin(th_1)))) / 8;

        if(abs(tmp) > 1){
            s[0] = 0;   s[1] = 0;   s[2] = 0;
            return false; 
        }
        s[1] = inv_k * mod2pi(2*PI - acos(tmp));
        s[0] = inv_k * mod2pi(atan2(C, S) + k*s[1]/2 - th_0);
        s[2] = inv_k * mod2pi(th_1 - th_0 + k*(s[1] - s[0]));
        
        double _k[3];
        _k[0] = 1;  _k[1] = -1;   _k[2] = 1;
        return check_val(s, _k, th_0, th_1, k);
    }

    bool check_val(double* s, double* k, double th0, double thf, double sc_k){

        double x0 = -1;
        double y0 = 0;
        double xf = 1;
        double yf = 0;

        k[0] *= sc_k; k[1] *= sc_k; k[2] *= sc_k;

        double eq1 =    // =2
            x0 + s[0] * sinc((0.50) * k[0] * s[0]) * cos(th0 + (0.5) * k[0] * s[0]) +
            s[1] * sinc((0.5) * k[1] * s[1]) * cos(th0 + (k[0] * s[1]) + ((0.5) * k[1] * s[1])) +
            s[2] * sinc((0.5) * k[2] * s[2]) * cos(th0 + (k[0] * s[2]) + (k[1] * s[2]) + ((0.5) * k[2] * s[2])) - xf;
        
        double eq2 =  y0 +    // =0
            (s[0] * sinc((0.5) * k[0] * s[0]) * sin(th0 + ((0.5) * k[0] * s[0]))) +
            (s[1] * sinc((0.5) * k[1] * s[1]) * sin(th0 + (k[0] * s[0]) + ((0.5) * k[1] * s[1]))) +
            (s[2] * sinc((0.5) * k[2] * s[2]) * sin(th0 + (k[0] * s[0]) + (k[1] * s[1]) + ((0.5) * k[2] * s[2]))) - yf;
        
        double eq3 =    // =thf
            rangeSymm(k[0] * s[0] + k[1] * s[1] + k[2] * s[2] + th0 - thf);

        bool Lpos = (s[0] > 0) || (s[1] > 0) | (s[2] > 0);

        return Lpos;
        //return ((eq1 * eq1) + (eq2 * eq2) + (eq3 * eq3) < 0.1) && Lpos;
    }

    void move_left(int duration){
        if(duration > ((2*PI - tolerance) * 1000 / w_)){
            // RCLCPP_INFO(this->get_logger(), "NO LEFT MOVEMENT %d", duration);
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "left for %d", duration);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = v_ang_;
        msg.angular.z = w_;
        cmd_vel_->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(duration));

        msg.angular.z = 0;
        msg.linear.x = 0;
        cmd_vel_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "LEFT STOPPED");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void move_right(int duration){
        if(duration > ((2*PI - tolerance) * 1000 / w_) - tolerance){
            // RCLCPP_INFO(this->get_logger(), "NO RIGHT MOVEMENT %d", duration);
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "right for %d", duration);
        geometry_msgs::msg::Twist msg;
        msg.linear.x = v_ang_;
        msg.angular.z = -w_;
        cmd_vel_->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(duration));

        msg.linear.x = 0;
        msg.angular.z = 0;
        cmd_vel_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "RIGHT STOPPED");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void move_straight(int duration){
        // RCLCPP_INFO(this->get_logger(), "straight for %d", duration);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = v_;
        msg.angular.z = 0;
        cmd_vel_->publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(duration));

        msg.linear.x = 0;
        cmd_vel_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "STRAIGHT STOPPED");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Dubins>();

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}