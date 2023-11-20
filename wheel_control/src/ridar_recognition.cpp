#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class ObstacleAvoidance
{
public:
    ObstacleAvoidance()
    {
        // パブリッシャーとサブスクライバーの設定
        laser_sub_ = nh_.subscribe("/scan", 1, &ObstacleAvoidance::laserCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // 初期化
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.linear.y = 0.0;
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.angular.x = 0.0;
        cmd_vel_.angular.y = 0.0;
        cmd_vel_.angular.z = 0.0;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // LiDARデータ処理
        std::vector<int> obstacle_indices = findClosestObstacles(msg, num_obstacles_);

        // 障害物の中心に向かって移動
        moveTowardObstacleCenter(obstacle_indices, msg);

        // パブリッシュ
        cmd_vel_pub_.publish(cmd_vel_);
    }

    std::vector<int> findClosestObstacles(const sensor_msgs::LaserScan::ConstPtr& msg, int num_obstacles)
    {
        // 最も近いN個の障害物のインデックスを見つける
        std::vector<int> obstacle_indices;
        std::vector<float> ranges_copy = msg->ranges;
        
        for (int i = 0; i < std::min(num_obstacles, static_cast<int>(ranges_copy.size())); ++i)
        {
            int min_index = std::min_element(ranges_copy.begin(), ranges_copy.end()) - ranges_copy.begin();
            obstacle_indices.push_back(min_index);
            ranges_copy[min_index] = std::numeric_limits<float>::infinity(); // 見つかった障害物を無効化
        }

        return obstacle_indices;
    }

    void moveTowardObstacleCenter(const std::vector<int>& obstacle_indices, const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // 障害物の中心に向かって移動するベクトルを計算
        double center_angle = calculateCenterAngle(obstacle_indices, msg);
        cmd_vel_.linear.x = linear_velocity_;
        cmd_vel_.angular.z = angular_velocity_factor_ * center_angle;
    }

    double calculateCenterAngle(const std::vector<int>& obstacle_indices, const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // 障害物の中心に向かって移動するための角度を計算
        double center_angle = 0.0;

        for (int index : obstacle_indices)
        {
            double angle_increment = msg->angle_increment;
            double angle_min = msg->angle_min;

            center_angle += (angle_min + index * angle_increment);
        }

        center_angle /= obstacle_indices.size();

        return center_angle;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist cmd_vel_;
    int num_obstacles_ = 50; // 考慮する障害物の数

    // パラメータ
    double linear_velocity_ = 0.2;         // 直進速度
    double angular_velocity_factor_ = 0.5;  // 角速度の調整係数
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");

    ObstacleAvoidance obstacle_avoidance;

    ros::spin();

    return 0;
}