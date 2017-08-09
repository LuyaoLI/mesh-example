#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main (int argc, char** argv)
{
  ros::init (argc, argv, "load_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud>("points", 10);

  PointCloud::Ptr cloud (new PointCloud);
  std::string pcd_filepath = "/home/chaohui/cloud_corrected.pcd";
  if (argc > 1)
    pcd_filepath = argv[1];
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filepath, *cloud) == -1) {
    PCL_ERROR ("Couldn't read file cloud_corrected.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from cloud\n"
            << std::endl;
  
  ros::Rate loop_rate(10);
  cloud->header.frame_id ="map";
  while (nh.ok()) {
    pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
    pub.publish(cloud);
    loop_rate.sleep();
  }

  return (0);
}