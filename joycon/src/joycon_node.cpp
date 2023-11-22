#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
// #include <std_msgs/Bool.h>

// 0 ==  auto
// 1 == manual


class JoyTwist
{
    public:
    // コンストラクタ
        JoyTwist()
        {
            ros::NodeHandle node;
            mode.data = 1;

            // 
            // joconの信号を受け取る
            joy_sub_ = node.subscribe("joy", 10, &JoyTwist::joyCallback, this);

            // 0 ==  auto
            // 1 == manual
            // mode.data = 0;
            twist_pub_ = node.advertise<geometry_msgs::Twist>("cmd_vel_joy", 1);
            mode_pub_ = node.advertise<std_msgs::Int32>("mode", 1);

        }
        //コールバック関数
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
        {
            //geometry_msgs::Twist twist; // Twistメッセージを保持
    
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
            twist.linear.y = joy_msg->axes[0]*2.0;
            twist.linear.x = joy_msg->axes[1]*2.0;
            twist.angular.z = joy_msg->axes[2]*5.0;

            // mode change
            ROS_INFO("L: %d R: %d", joy_msg->buttons[4],joy_msg->buttons[5]);
            if((joy_msg->buttons[4] == 1) && (joy_msg->buttons[5] == 1)){
                ROS_INFO("LR_pushed");
                if(mode.data == 1)mode.data = 2;
                else if(mode.data == 2)mode.data = 1;
            }
        
    
            // 左スティックで前後の移動
            // 右スティックで左右の移動
            // L,Rボタンで回転
            // twist.linear.x = - joy_msg->axes[2];
            



            //twist_pub_.publish(twist);
            // mode_pub_.publish(mode);
            
            // コントローラの値を表示
            // ROS_INFO("y: %f", joy_msg->axes[1]);
            // ROS_INFO("x: %f", joy_msg->axes[2]);
        }

            // cmd_velを定期的にパブリッシュする
        void publishTwist() {
            twist_pub_.publish(twist);
            mode_pub_.publish(mode);
        }


    private:
        ros::Subscriber joy_sub_;
        ros::Publisher twist_pub_;
        ros::Publisher mode_pub_;
        geometry_msgs::Twist twist; // Twistメッセージを保持
        std_msgs::Int32 mode;
        
};

int main(int argc, char **argv)
{
    // nodeを定義
    ros::init(argc, argv, "joy_twist");
    JoyTwist joy_twist;

    ros::Rate loop_rate(10); // 10Hzでループ
    while (ros::ok()) {
        joy_twist.publishTwist(); // Twistメッセージをパブリッシュ
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
    
}
