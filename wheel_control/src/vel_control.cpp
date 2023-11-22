#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <math.h>

class PIDController_anguler
{
private:
    float kp_; // 比例ゲイン
    float ki_; // 積分ゲイン
    float kd_; // 微分ゲイン

    float integral_;
    float pre_error_;

public:
    PIDController_anguler(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), pre_error_(0.0) {}

    float compute(float setpoint, float pv)
    {
        float error = pv - setpoint;
        integral_ += error;
        float derivative = error - pre_error_;
        pre_error_ = error;
        ROS_INFO("error: %f,pv: %f,setpoint: %f,signal %f", error,pv,setpoint,kp_ * error + ki_ * integral_ + kd_ * derivative);
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
};

class PIDController_linear
{
private:
    float kp_; // 比例ゲイン
    float ki_; // 積分ゲイン
    float kd_; // 微分ゲイン

    float integral_;
    float pre_error_;

public:
    PIDController_linear(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), pre_error_(0.0) {}

    float compute(float setpoint, float pv)
    {
        float error = pv - setpoint;
        ROS_INFO("error: %f,pv: %f,setpoint: %f", error,pv,setpoint);
        integral_ += error;
        float derivative = error - pre_error_;
        pre_error_ = error;
        integral_ = 0;
        // derivative = 0;
        kp_ = 1.0;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }
};

class PointFollower
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher cmd_pub_;
    geometry_msgs::Point current_position_; // ロボットの現在位置
    PIDController_linear pid_linear;
    PIDController_anguler pid_anguler;

    void pointCallback(const geometry_msgs::Point::ConstPtr& msg)
    {   current_position_.x = 0.0;
        current_position_.y = 0.0;
        // ターゲットまでの距離を計算
        float distance = sqrt(pow(msg->x - current_position_.x, 2) + pow(msg->y - current_position_.y, 2));
        float target_x,target_y,target_angle;
        target_x = msg->x*(-1);
        target_y = msg->y*(-1);
        target_angle = atan2(target_x,target_y*(-1))*(180/3.1415);
        // PID制御
        float control_signal_linear = pid_linear.compute(0.3, distance); // 5cm = 0.05m
        float control_signal_anguler = pid_anguler.compute(90.0, target_angle); // 5cm = 0.05m

        // Twistメッセージを生成
        geometry_msgs::Twist cmd_msg;
        if(distance==0){
            cmd_msg.angular.z = 0.5;
        }else if((target_angle < 80)||(target_angle > 100)){
            // if(target_angle < 80)cmd_msg.angular.z = -0.8*control_signal_anguler;
            // else if(target_angle > 100)cmd_msg.angular.z = 0.8*control_signal_anguler;
            cmd_msg.linear.x = target_x*control_signal_linear*7.0; 
            cmd_msg.linear.y = target_y*control_signal_linear*7.0;
            cmd_msg.angular.z = 1.0*control_signal_anguler;
        }else{
            cmd_msg.linear.x = target_x*control_signal_linear*7.0; 
            cmd_msg.linear.y = target_y*control_signal_linear*7.0;
            cmd_msg.angular.z = 0.0;
        }
        cmd_pub_.publish(cmd_msg);
        ROS_INFO("distance: %f, control_signal: %f,x: %f,y: %f angle: %f", distance, control_signal_anguler,target_y*(-1),target_x, target_angle);
    }

public:
    PointFollower() : pid_linear(0.1, 0.01, 0.5) ,pid_anguler(0.01, 0.0, 0.0)// PIDゲインは調整する
    {
        point_sub_ = nh_.subscribe<geometry_msgs::Point>("target_point", 10, &PointFollower::pointCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_auto", 10);
        // current_position_ の初期化が必要
    }

    // current_position_ を更新するためのメソッドが必要
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_follower");
    PointFollower follower;
    ros::spin();
    return 0;
}

