#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <cctype>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef geometry_msgs::Point Point;
typedef visualization_msgs::MarkerArray MarkerArray;
typedef visualization_msgs::Marker Marker;

double centerX = 0.0;
double centerY = 0.0;

// Make basic marker
Marker makeMarker(int id, std::string ns, double scale, int r, int g, int b) {
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time(0);
  marker.ns = ns;
  marker.id = id;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  return marker;
}

// Make white text marker
Marker makeText(int id, double x, double y, double z, std::string s) {
  Marker text = makeMarker(id, "hull", 0.6, 1.0, 1.0, 1.0);
  text.type = Marker::TEXT_VIEW_FACING;
  text.pose.position.x = x;
  text.pose.position.y = y;
  text.pose.position.z = z;
  text.text = s;
  return text;
}

// Make line strip marker to outline convex hulls
Marker makeOutline(int id, int r, int g, int b) {
  Marker line_strip = makeMarker(id, "hull", 0.03, r, g, b);
  line_strip.type = Marker::LINE_STRIP;
  return line_strip;
}

// Make red cube marker to represent convex hull vertices
Marker makeVertex(int id, double x, double y, double z) {
  Marker cube = makeMarker(id, "vertices", 0.03, 1, 0, 0);
  cube.type = Marker::CUBE;
  cube.pose.position.x = x;
  cube.pose.position.y = y;
  cube.pose.position.z = z;
  return cube;
}

// Make geometry_msgs::Point
Point makePoint(double x, double y, double z) {
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// Returns true if clockwise angle from (centerX, centerY) to point a is less
// than the clockwise angle from center to point b, false otherwise
bool compareClockwise(Point a, Point b) {
  double angleA = atan2(-(a.y-centerY), a.x-centerX);
  double angleB = atan2(-(b.y-centerY), b.x-centerX);
  return angleA < angleB;
}

// Returns true if point p lies on segment s1s2, false otherwise
bool onSegment(Point p, Point s1, Point s2) {
  double xMin = std::min(s1.x, s2.x);
  double xMax = std::max(s1.x, s2.x);
  double yMin = std::min(s1.y, s2.y);
  double yMax = std::max(s1.y, s2.y);
  if (xMin <= p.x && p.x <= xMax && yMin <= p.y && p.y <= yMax)
    return true;
  return false;
}

// Returns true if segment p1p2 intersects edge e1e2, false otherwise
bool intersectsEdge(Point p1, Point p2, Point e1, Point e2) {
  double a = (p2.y - p1.y) / (p2.x - p1.x);
  double b = (e2.y - e1.y) / (e2.x - e1.x);
  if (std::abs(a - b) <= 0.0001) 
    return false;

  double c = p1.y - (p2.y - p1.y) / (p2.x - p1.x) * p1.x;
  double d = e1.y - (e2.y - e1.y) / (e2.x - e1.x) * e1.x;
  double x = (d - c) / (a - b);
  double y = a * (d - c) / (a - b) + c;
  Point intersection = makePoint(x, y, 0.0);
  if (onSegment(intersection, p1, p2) && onSegment(intersection, e1, e2))
    return true;
  return false;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "extract_clusters");
  ros::NodeHandle nh;

  // Load point cloud from file
  PointCloud::Ptr cloud(new PointCloud);
  std::string pcd_filepath = argv[3];
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_filepath, *cloud) == -1) {
    PCL_ERROR ("Couldn't read file cloud_corrected.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height
            << " data points from cloud_corrected.pcd\n" << std::endl;

  // Creating KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  // Extract clusters from point cloud
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // Higher the tolerance, the less clusters
  // Adjust cluster tolerance and min/max cluster size as needed
  ec.setClusterTolerance (0.075);
  ec.setMinClusterSize (400);
  ec.setMaxClusterSize (190000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  // Create set of planar coefficients with X=Y=0, Z=1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  // Create filtering object to project a cluster to the XY plane
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setModelCoefficients(coefficients);
  pcl::ConvexHull<pcl::PointXYZ> chull;

  // Average and minimum x and y values of each hull's vertices
  std::vector<std::vector<double> > hullBoundary;
  // Vertices of each convex hull
  std::vector<std::vector<Point> > hullVertices;

  // Point cloud clusters and their respective ROS publishers
  std::vector<PointCloud> cloudList;
  std::vector<ros::Publisher> pubList;

  int id = 0;
  int numClusters = 0;
  MarkerArray vertexArray;
  MarkerArray hullArray;
  ros::Publisher hullPub = nh.advertise<MarkerArray>("hulls", 10);
  ros::Publisher vertexPub = nh.advertise<MarkerArray>("vertices", 10);

  // Add indices from exclude.txt of hulls to exclude from being obstacles
  std::vector<int> excludeHull;
  if (std::string(argv[2]) == "true") {
    int index;
    std::ifstream excludeFile(argv[5]);
    while (excludeFile >> index)
      excludeHull.push_back(index);
  }

  // Add each cluster and its publisher to cloudList and pubList, construct its
  // convex hull, sort the hull vertices in clockwise order, and add the
  // vertices and boundary to hullVertices and hullBoundary
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
        it != cluster_indices.end(); ++it) {
    PointCloud::Ptr cloud_cluster (new PointCloud);
    // Get next cloud cluster
    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
          pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::ostringstream oss;
    oss << numClusters;
    std::string topic = "cloud" + oss.str();
    pubList.push_back(nh.advertise<PointCloud>(topic, 10));

    // Down-project current cluster to XY plane
    PointCloud::Ptr cloud_projected (new PointCloud);
    proj.setInputCloud(cloud_cluster);
    proj.filter(*cloud_projected);
    cloud_projected->header.frame_id = "map";
    cloudList.push_back(*cloud_projected);

    // Construct convex hull of cloud_projected
    PointCloud::Ptr cloud_hull (new PointCloud);
    chull.setInputCloud(cloud_projected);
    chull.reconstruct(*cloud_hull);
    numClusters++;

    std::vector<Point> vertices;
    std::vector<double> boundary;
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    // Get minimum and maximum X, Y, Z coordinates of convex hull
    pcl::getMinMax3D(*cloud_hull, min, max);
    // Center of hull as average of vertices
    centerX = 0.0;
    centerY = 0.0;

    for (int j = 0; j < cloud_hull->points.size(); j++) {
      double x = roundf((cloud_hull->points[j].x) * 1000) / 1000;
      double y = roundf((cloud_hull->points[j].y) * 1000) / 1000;
      Point p = makePoint(x, y, 0.0);
      centerX += x;
      centerY += y;
      vertices.push_back(p);
    }
    centerX /= cloud_hull->points.size();
    centerY /= cloud_hull->points.size();
    // Sort convex hull vertices in clockwise order around hull's center
    std::sort(vertices.begin(), vertices.end(), compareClockwise);

    boundary.push_back(centerX);
    boundary.push_back(centerY);
    boundary.push_back(min.x);
    boundary.push_back(max.x);
    boundary.push_back(min.y);
    boundary.push_back(max.y);
    hullBoundary.push_back(boundary);
    hullVertices.push_back(vertices);
  }

  // Check if a hull's center is inside any other hulls; if so, don't include
  // it as an obstacle
  for (int i = 0; i < numClusters; i++) {
    Point center = makePoint(hullBoundary[i][0], hullBoundary[i][1], 0.0);
    bool inside = false;
    for (int j = 0; j < numClusters; j++) {
      if (j != i) {
        double xMin = hullBoundary[j][2];
        double xMax = hullBoundary[j][3];
        double yMin = hullBoundary[j][4];
        double yMax = hullBoundary[j][5];
        if (center.x >= xMin && center.x <= xMax && 
            center.y >= yMin && center.y <= yMax) {
          double epsilon = std::abs(xMax - xMin) / 100;
          Point p2 = makePoint(xMax + epsilon, center.y, 0.0);
          // Count number of hull j's edges intersected by the line segment from
          // center of hull i to point p2
          std::vector<Point> vertices = hullVertices[j];
          int count = 0;
          for (int v = 0; v < vertices.size(); v++) {
            int next = v+1;
            if (v == vertices.size() - 1)
              next = 0;
            if (intersectsEdge(center, p2, vertices[v], vertices[next]))
              count++;
          }
          // If odd number of edges intersected, center of hull i is inside hull j
          if (count % 2 == 1) {
            inside = true;
            break;
          }
        }
      }
    }
    if (inside)
      excludeHull.push_back(i);
  }
  
  bool writeObstacles = true;
  if (std::string(argv[1]) == "false")
    writeObstacles = false;

  std::ofstream myfile;
  if (writeObstacles) {
    myfile.open(argv[4]);
    myfile << "0.65\n";
    myfile << hullVertices.size() - excludeHull.size() << "\n";
  }

  for (int i = 0; i < hullVertices.size(); i++) {
    if (std::find(excludeHull.begin(), excludeHull.end(), i) == excludeHull.end()) {
      std::vector<Point> vertices = hullVertices[i];
      std::vector<Point>::iterator itV;
      Marker outline = makeOutline(id, 1, 0, 0);
      id++;
      myfile << vertices.size() << "\n";
      for (itV=vertices.begin(); itV!=vertices.end(); itV++) {
        if (writeObstacles)
          myfile << (*itV).x << " " << -(*itV).y << "\n";
        vertexArray.markers.push_back(makeVertex(id, (*itV).x, (*itV).y, 0.0));
        id++;
        outline.points.push_back(*itV);
      }
      std::ostringstream oss;
      oss << i;
      hullArray.markers.push_back(makeText(id, hullBoundary[i][0],
                                  hullBoundary[i][1], 0.0, oss.str()));
      id++;
      outline.points.push_back(vertices.front());
      hullArray.markers.push_back(outline);
    }
  }
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(*cloud, min, max);
  Marker outline = makeOutline(id, 0, 1, 0);
  outline.points.push_back(makePoint(min.x, min.y, 0.0));
  outline.points.push_back(makePoint(min.x, max.y, 0.0));
  outline.points.push_back(makePoint(max.x, max.y, 0.0));
  outline.points.push_back(makePoint(max.x, min.y, 0.0));
  outline.points.push_back(makePoint(min.x, min.y, 0.0));
  hullArray.markers.push_back(outline);

  if (writeObstacles) {
    myfile << "4\n";
    myfile << min.x << " " << -max.y << "\n";
    myfile << min.x << " " << -min.y << "\n";
    myfile << max.x << " " << -min.y << "\n";
    myfile << max.x << " " << -max.y;
    std::cout << "Wrote to obstacles.txt in home directory\n" << std::endl;
    myfile.close();
  }
  std::cout << "To exclude certain hulls from being an obstacle, write the "
            << "number of the hull (visible in rviz) in a file called exclude.txt, "
            << "one obstacle per line. Then run extract_clusters.launch with "
            << "exclude_obstacles:= true and exclude_filepath:=path_to_exclude.txt"
            << std::endl;

  ros::Rate loop_rate(1);
  while (nh.ok()) {
    for (int i = 0; i < hullVertices.size(); i++) {
      if (std::find(excludeHull.begin(), excludeHull.end(), i) == excludeHull.end()) {
        pcl_conversions::toPCL(ros::Time::now(), cloudList[i].header.stamp);
        pubList[i].publish(cloudList[i]);
      }
    }
    vertexPub.publish(vertexArray);
    hullPub.publish(hullArray);
    loop_rate.sleep();
  }

  return (0);
}