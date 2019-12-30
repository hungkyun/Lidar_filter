/*
  author: yxt
*/
#include "lidar_main.h"

using namespace Eigen;
using namespace std;

void readPCDfile(const std::string finname, std::vector<cv::Point3d>& points);
void lidarPointsFilter(std::vector<cv::Point3d>& lidar_poitns);
void filtering(std::vector<cv::Point3d>& lidar_points,std::vector<int>& labels, std::vector<cv::Point2f>& centers);

class Lidar_node{
public:
    Lidar_node();
    void process1(vector<int>& labels, vector<cv::Point2f>& centers);
	void process2();
	void clustering(pcl::PointCloud<pcl::PointXYZ> points, pcl::PointCloud<pcl::PointXYZ> &result);
	float p2pdis(pcl::PointXYZ a, pcl::PointXYZ b);
private:
    ros::NodeHandle node_handle_;
    ros::Publisher pub;
    ros::Publisher fpub;
    int step;
    int maxdis;
    int min_cluster_number;
    int max_distance_in_cluster;
};

Lidar_node::Lidar_node()
{
	pub = node_handle_.advertise<sensor_msgs::PointCloud2 >("point",10);
	fpub = node_handle_.advertise<sensor_msgs::PointCloud2 >("filter",10);
	step = 2;
	maxdis = 500;  // the maximum distance that lidar can detect
	min_cluster_number = 8;
	max_distance_in_cluster = 5;
}

// cluster function
void Lidar_node::clustering(pcl::PointCloud<pcl::PointXYZ> points, pcl::PointCloud<pcl::PointXYZ> &result) {
	vector<vector<int>> filter;
	for (int k = 0; k < maxdis/step; ++k) {
		filter.push_back(vector<int>());
	}
	pcl::PointXYZ origin(0,0,0);
	for (int j = 0; j < points.size(); ++j) {
		int index = (int)p2pdis(origin,points[j]);
		filter[index].push_back(j);
	}

	for (int l = 0; l < maxdis / step; ++l) {
		int num = filter[l].size();
		if(num > min_cluster_number){
			float maxdistance = -999;
			for (int m = 0; m < 10; ++m) {
				int m1 = rand()%filter[l].size();
				int m2 = rand()%filter[l].size();
				float dis = p2pdis(points[filter[l][m1]],points[filter[l][m2]]);
				if(dis>maxdistance)
					maxdistance = dis;
			}
			if(maxdistance < max_distance_in_cluster){
				for (int j = 0; j < filter[l].size(); ++j) {
					result.push_back(points[filter[l][j]]);
				}
			}
		}
	}
}

float Lidar_node::p2pdis(pcl::PointXYZ a, pcl::PointXYZ b) {
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
}


void print(pcl::PointXYZ p){
	cout << p.x << " " << p.y << " " << p.z << endl;
}



void Lidar_node::process1(vector<int>& labels, vector<cv::Point2f>& center){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i < 150; i++){
		ifstream in;
		string str = "/home/yxt/data/yunzhou/data1221-1/01/lidar/";
		stringstream ss;
		string num;
		ss.clear();
		ss << i;
		ss >> num;
		str = str + num + ".pcd";
		if(pcl::io::loadPCDFile<pcl::PointXYZ>(str, *cloud)==-1){
			cout << "error " << endl;
			return;
		}


		/*
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
		for (int j = 0; j < cloud->points.size(); ++j) {
			if(cloud->points[j].x >= 0){
				cloud2->points.push_back(cloud->points[j]);
			}
		}
		*/
		cout << str << endl;
		sensor_msgs::PointCloud2 pp;
		pcl::toROSMsg(*cloud,pp);
		pp.header.frame_id = "pandar";
		pub.publish(pp);
		sleep(1);
	}

}

void Lidar_node::process2() {

	for (int i = 0; i < 115; ++i)
	{


		ifstream in;
		string str = "/home/yxt/conference/yunzhou/detect_results/";
		stringstream ss;
		string num;
		ss.clear();
		ss << i;
		ss >> num;
		str = str + num + ".txt";
		cout << "open " << str << endl;
		in.open(str.data());
		string s;
		pcl::PointCloud<pcl::PointXYZ>::Ptr p(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pend(new pcl::PointCloud<pcl::PointXYZ>);

		while(getline(in,s))
		{
			pcl::PointXYZ point;
			stringstream aa;
			aa.clear();
			aa << s;
			aa >> point.x >> point.y >> point.z;
			p->points.push_back(point);
		}
		pcl::PointCloud<pcl::PointXYZ> result;
		clustering(*p,result);

		sensor_msgs::PointCloud2 pp;
		pcl::toROSMsg(*p,pp);
		pp.header.frame_id = "velodyne";
		pub.publish(pp);

		sensor_msgs::PointCloud2 fresult;
		pcl::toROSMsg(result,fresult);
		fresult.header.frame_id = "velodyne";
		fpub.publish(fresult);

		sleep(1);
	}
}

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
    srand(time(NULL));
    ros::init(argc,argv,"Lidar_node");
    Lidar_node node;
	main_int8();


    //node.process2();
	//node.process1();
    return 0;
}
