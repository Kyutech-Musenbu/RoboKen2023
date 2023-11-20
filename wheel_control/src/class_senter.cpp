#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <geometry_msgs/Point.h>


class LaserScanToPointCloud
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    ros::Publisher point_pub_;
    laser_geometry::LaserProjection projector_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        sensor_msgs::PointCloud2 cloud;
        projector_.projectLaser(*scan, cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud, *pcl_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pcl_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.08); // 2cm
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_cloud);
        ec.extract(cluster_indices);
        ROS_INFO("Number of clusters found: %ld", cluster_indices.size());

        float distance = 0, min_distance = 100;
        float min_centroid[3] = {0,0,0};
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : indices.indices)
                cloud_cluster->push_back((*pcl_cloud)[idx]);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            ROS_INFO("Cluster centroid: %f, %f, %f", centroid[0], centroid[1], centroid[2]);
            float distance = sqrt(pow(centroid[0], 2) + pow(centroid[1], 2));
            if(distance<min_distance){
                min_distance = distance;
                min_centroid[0] = centroid[0];
                min_centroid[1] = centroid[1];
                min_centroid[2] = centroid[2];
            }
        }

        geometry_msgs::Point point_msg;
        point_msg.x = min_centroid[0];
        point_msg.y = min_centroid[1];
        point_msg.z = min_centroid[2];
        point_pub_.publish(point_msg);

        // オプションで変換されたPointCloud2メッセージを公開
        cloud_pub_.publish(cloud);
    }

public:
    LaserScanToPointCloud()
    {
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 10, &LaserScanToPointCloud::scanCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("converted_cloud", 1);
        point_pub_ = nh_.advertise<geometry_msgs::Point>("target_point", 1);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_to_point_cloud");
    LaserScanToPointCloud node;
    ros::spin();
    return 0;
}

