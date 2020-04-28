#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
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
	cv::Point begin,end;
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
	int point_num;
	int age;
	cv::Point target_point;
	cv::Mat param;
	cv::Mat pred;
	cv::KalmanFilter kalman;
	Target_(){
		age = 0;
		trace_ID = -1;
		match = 0;
		point_num = 0;
	}
	Vertex vertex;
};


class KM{
public:
	KM();
	bool dfs(int first);
	void km_init();
	double ikm_match();
	const static int maxn=20;
	const static int inf=999999;
	bool vis_first[maxn];
	bool vis_second[maxn];
	double mmp[maxn][maxn];
	double slack[maxn];
	double val_first[maxn];
	double val_second[maxn];
	int match[maxn];
	int num;
};

KM::KM(){
	km_init();
}

void KM::km_init() {
	num = 0;
	memset(mmp,0,sizeof(mmp));
	memset(match,0,sizeof(match));
	memset(slack,0,sizeof(slack));
	memset(match,-1,sizeof(match));
	memset(val_first,0,sizeof(val_first));
	memset(val_second,0,sizeof(val_second));
	memset(vis_first,false,sizeof(vis_first));
	memset(vis_second,false,sizeof(vis_second));
}

double KM::ikm_match() {
	memset(match,-1,sizeof(match));
	memset(val_second,0,sizeof(val_second));
	for(int i=0;i<num;i++){
		val_first[i]=mmp[i][0];
		for(int j=1;j<num;j++){
			val_first[i]=cv::max(val_first[i],mmp[i][j]);
		}
	}
	for(int i=0;i<num;i++){
		for (int k = 0; k < num; ++k) {
			slack[k] = inf*1.0;
		}
		while(1){
			memset(vis_first,false,sizeof(vis_first));
			memset(vis_second,false,sizeof(vis_second));
			if(dfs(i)) break;
			double v=inf;
			for(int j=0;j<num;j++)if(!vis_second[j]) v=cv::min(v,slack[j]);
			for(int j=0;j<num;j++){
				if(vis_first[j]) val_first[j]-=v;
				if(vis_second[j]) val_second[j]+=v;
				else slack[j]-=v;
			}
		}
	}
	double ans=0.0;
	for(int i=0;i<num;i++) ans+=mmp[match[i]][i];
	return ans;
}

bool KM::dfs(int first) {
	vis_first[first]=true;
	for(int j=0;j<num;j++){
		if(vis_second[j]) continue;
		double x=val_first[first]+val_second[j]-mmp[first][j];
		if(abs(x)<0.0000001){
			vis_second[j]=true;
			if(match[j]==-1||dfs(match[j])){
				match[j]=first;
				return true;
			}
		}
		else{
			slack[j]=cv::min(slack[j],x);
		}
	}
	return false;
}