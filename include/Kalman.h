#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


struct Vertex{
	cv::Point upper,lower;
	cv::Point right_point,left_point;
	cv::Point top_point,low_point;
	cv::Point mid_point;
	int flag;
	double longth;
	Vertex(){
		flag = -1;
		longth = 0.0;
	}
};

struct Target_{
	int trace_ID;
	int match;
	cv::Mat param;
	cv::Point target_point;
	cv::Mat pred;
	cv::KalmanFilter kalman;
	Target_(){
		trace_ID = -1;
		match = 0;
	}
	Vertex vertex;
};

struct curr_tar{
	cv::Mat para;
	int remove;
	curr_tar(){
		remove = 0;
	}
};

