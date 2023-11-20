#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>


class PointToArrowVisualizer
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber point_sub_;
    ros::Publisher marker_pub_;

    void pointCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser";  // 適切なフレームに変更してください
        marker.header.stamp = ros::Time::now();
        marker.ns = "point_to_arrow";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 矢印の始点と終点を設定（ここでは始点を原点、終点を受け取った点とする）
        geometry_msgs::Point start, end;
        start.x = 0; start.y = 0; start.z = 0;
        end = *msg;
        marker.points.push_back(start);
        marker.points.push_back(end);

        // 矢印のスケール（幅）と色を設定
        marker.scale.x = 0.1; // 矢印の幅
        marker.scale.y = 0.2; // 矢印の先端の幅
        marker.color.a = 1.0; // アルファ値
        marker.color.r = 1.0; // 赤色
        marker.color.g = 0.0; // 緑色
        marker.color.b = 0.0; // 青色

        marker_pub_.publish(marker);
    }

public:
    PointToArrowVisualizer()
    {
        point_sub_ = nh_.subscribe<geometry_msgs::Point>("target_point", 10, &PointToArrowVisualizer::pointCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_to_arrow_visualizer");
    PointToArrowVisualizer visualizer;
    ros::spin();
    return 0;
}
