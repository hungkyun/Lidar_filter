/*
  author: yxt
*/
#include "lidar_main.h"

using namespace Eigen;
using namespace std;


class Lidar_node{
public:
    Lidar_node();
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

int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc,argv,"Lidar_node");
    Lidar_node node;
    node.process2();

    return 0;
}
