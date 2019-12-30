/*
  author: yxt
*/
#include "lidar_main.h"

using namespace Eigen;
using namespace std;

void readPCDfile(const std::string finname, std::vector<cv::Point3d>& points);
void lidarPointsFilter(std::vector<cv::Point3d>& lidar_poitns);
void filtering(std::vector<cv::Point3d>& lidar_points,std::vector<int>& labels, std::vector<cv::Point2f>& centers);


void readPCDfile(const std::string finname, std::vector<cv::Point3d>& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (finname, *cloud) == -1){
		return ;
	}

	for (size_t i = 0; i < cloud->points.size (); ++i){
		points.push_back(cv::Point3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z));
	}
}

void lidarPointsFilter(std::vector<cv::Point3d>& lidar_poitns)
{
	std::vector<cv::Point3d> filter;
	filter.reserve(lidar_poitns.size());
	for(auto it : lidar_poitns){
		if(it.y < 15)
			continue;
		filter.push_back(it);
	}
	lidar_poitns.swap(filter);
}

void cal_center(std::vector<cv::Point3d>& points, std::vector<cv::Point2f>& centers)
{
	cv::Point2f center(0,0);
	for (int i = 0; i < points.size(); ++i) {
		center.x += points[i].x;
		center.y += points[i].y;
	}
	center.x /= points.size();
	center.y /= points.size();
	centers.push_back(center);
}

void filtering(std::vector<cv::Point3d>& lidar_points, std::vector<int>& labels, std::vector<cv::Point2f>& centers)
{
	int clusternum = 0;
	float radius = 0.5; // 0.5m
	std::vector<bool> visited;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointscloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < lidar_points.size(); ++i) {
		visited.push_back(false);  // initialize
		labels.push_back(-1);
		pcl::PointXYZ point(lidar_points[i].x,lidar_points[i].y,lidar_points[i].z);
		pointscloud->points.push_back(point);
	}
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(pointscloud);

	for (int j = 0; j < pointscloud->points.size(); ++j) {
		if(visited[j]==true){
			continue;
		}
		std::vector<int> pointRadiusSearch;
		std::vector<float> pointRadiusSquareDistance;
		int value;
		value = kdtree->radiusSearch(pointscloud->points[j],radius,pointRadiusSearch,pointRadiusSquareDistance);
		if(value > 0){
			visited[j] = true;
			labels[j] = clusternum;
			std::vector<cv::Point3d> points;

			for (int k = 0; k < pointRadiusSearch.size(); ++k) {
				if(visited[pointRadiusSearch[k]]==true){
					continue;
				}
				else{
					visited[pointRadiusSearch[k]] = true;
					labels[pointRadiusSearch[k]] = labels[j];
					points.push_back(lidar_points[pointRadiusSearch[k]]);
				}
			}
			cal_center(points,centers);
			clusternum += 1;
		}
	}
}

int main_int8()
{
	std::vector<cv::Point3d> lidar_poitns;
	readPCDfile("/home/yxt/data/yunzhou/data1221-1/01/lidar/0.pcd",lidar_poitns);

	std::vector<int> labels;
	std::vector<cv::Point2f> centers;
	lidarPointsFilter(lidar_poitns);
	filtering(lidar_poitns,labels,centers);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Lidar_node");
	main_int8();

    return 0;
}
