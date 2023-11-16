#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class JoyTwist
{
    public:
    // コンストラクタ
        JoyTwist()
        {
            ros::NodeHandle node;
            // joconの信号を受け取る
            joy_sub_ = node.subscribe("joy", 10, &JoyTwist::joyCallback, this);

            // 
            twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        }
        //コールバック関数
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
        {
            geometry_msgs::Twist twist;

            // 左スティックで前後の移動
            // 右スティックで左右の移動
            // L,Rボタンで回転
            // twist.linear.x = - joy_msg->axes[2];
            // twist.linear.y =  joy_msg->axes[1];

            // if(joy_msg->buttons[4] == 1){
            //     twist.angular.z = 1;
            // }else if(joy_msg->buttons[5] == 1){
            //     twist.angular.z = -1;
            // }

            // 左スティックで前後左右の移動
            // 右スティックで回転
            twist.linear.y = joy_msg->axes[0];
            twist.linear.x = joy_msg->axes[1];

            twist.angular.z = joy_msg->axes[2];


            twist_pub_.publish(twist);
            
            // コントローラの値を表示
            // ROS_INFO("y: %f", joy_msg->axes[1]);
            // ROS_INFO("x: %f", joy_msg->axes[2]);
        }


    private:
        ros::Subscriber joy_sub_;
        ros::Publisher twist_pub_;

};

int main(int argc, char **argv)
{
    // nodeを定義
    ros::init(argc, argv, "joy_twist");
    JoyTwist joy_twist;
    ros::spin();
    return 0;
}