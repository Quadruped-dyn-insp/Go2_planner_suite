#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class VoxelGridNode : public rclcpp::Node
{
public:
  VoxelGridNode() : Node("voxel_grid_node")
  {
    // Declare parameter for voxel size
    this->declare_parameter("voxel_size", 0.1f);

    // Create subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/dlio/map_node/map", 10, std::bind(&VoxelGridNode::topic_callback, this, std::placeholders::_1));

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan_o3d/voxelized", 10);

    RCLCPP_INFO(this->get_logger(), "VoxelGridNode started using voxel size: %.2f", this->get_parameter("voxel_size").as_double());
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pointcloud with %d x %d points", msg->width, msg->height);

    // 1. Convert ROS msg to PCL data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);
    
    // 2. Perform VoxelGrid filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    
    float voxel_size = this->get_parameter("voxel_size").as_double();
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud_filtered);

    // 3. Convert back to ROS msg
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    
    // Maintain the header frame and timestamp
    output.header = msg->header;
    
    // 4. Publish
    publisher_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridNode>());
  rclcpp::shutdown();
  return 0;
}
