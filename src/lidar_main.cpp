/*
  author: yxt
*/
#include "lidar_main.h"

using namespace Eigen;
using namespace std;


class Lidar_node{
public:
    Lidar_node();
    void cluster(pcl::PointCloud<pcl::PointXYZ> p, pcl::PointCloud<pcl::PointXYZ> &result);
	void process();
	void process2();
	float p2pdis(pcl::PointXYZ a, pcl::PointXYZ b);
private:
    ros::NodeHandle node_handle_;
    ros::Publisher pub;
    ros::Publisher fpub;
    pcl::PointCloud<pcl::PointXYZ> preframe_points;
    float radius;
    bool isFirstFrame;
    int step;
    int maxdis;
};

Lidar_node::Lidar_node()
{
	pub = node_handle_.advertise<sensor_msgs::PointCloud2 >("point",10);
	fpub = node_handle_.advertise<sensor_msgs::PointCloud2 >("filter",10);
	isFirstFrame = true;
	radius = 1.5;
	step = 2;
	maxdis = 500;
}

float Lidar_node::p2pdis(pcl::PointXYZ a, pcl::PointXYZ b) {
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z));
}

void Lidar_node::cluster(pcl::PointCloud<pcl::PointXYZ> p, pcl::PointCloud<pcl::PointXYZ> &result) {
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	cout << "..." << p.size() << endl;
	sor.setInputCloud(p.makeShared());
	sor.setMeanK(3);
	sor.setStddevMulThresh(0.5);
	sor.filter(result);
}

/*
void Lidar_node::process()
{
	pcl::PointCloud<pcl::PointXYZ> points[3];
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
			//cout << point.x << " " << point.y << " " << point.z << endl;
		}

		if(p->points.size()==0){
			isFirstFrame = true;
			continue;
		}


		pcl::PointCloud<pcl::PointXYZ> result;
		if(isFirstFrame==false)
		{
			int *index = new int[p->points.size()];
			for (int k = 0; k < p->points.size(); ++k) { index[k] = 0; }

			for (int j = 0; j < preframe_points.size(); ++j) {
				for (int k = 0; k < p->points.size(); ++k) {
					float dis = p2pdis(preframe_points[j],p->points[k]);
					cout << dis << endl;
					if(dis <= radius){
						index[k] += 1;
					}
				}
			}

			for (int l = 0; l < p->points.size(); ++l) {
				if(index[l] > 1){
					result.push_back(p->points[index[l]]);
				}
			}
			cluster(result,preframe_points);
			delete []index;

		}

		cout << "///" << p->points.size() << endl;
		//density clustering
		if(isFirstFrame==true){
			cluster(*p,preframe_points);
			cout << i << " <cluster> " << preframe_points.size() << endl;
			isFirstFrame = false;
		}

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
 */

void Lidar_node::process2() {

	for (int i = 0; i < 115; ++i)
	{
		vector<vector<int>> filter;
		for (int k = 0; k < maxdis/step; ++k) {
			filter.push_back(vector<int>());
		}

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
		//cout << "num: " << p->points.size() << endl;
		pcl::PointXYZ origin(0,0,0);
		for (int j = 0; j < p->points.size(); ++j) {
			int index = (int)p2pdis(origin,p->points[j]);
			filter[index].push_back(j);
		}

		/*
		for (int l = 0; l < maxdis / step; ++l) {
			for (int j = 0; j < filter[l].size(); ++j) {
				cout << l << ": " << filter[l][j] << endl;
			}
		}
		*/

		pcl::PointCloud<pcl::PointXYZ> result;
		for (int l = 0; l < maxdis / step; ++l) {
			int num = filter[l].size();
			if(num>8){
				cout << "num: " << num << endl;
				float maxdistance = -999;
				for (int m = 0; m < 10; ++m) {
					int m1 = rand()%filter[l].size();
					int m2 = rand()%filter[l].size();
					float dis = p2pdis(p->points[m1],p->points[m2]);
					if(dis>maxdistance)
						maxdistance = dis;
				}
				cout << maxdistance << endl;
				//if(maxdistance<7){
					for (int j = 0; j < filter[l].size(); ++j) {
						result.push_back(p->points[filter[l][j]]);
					}
				//}
			}
		}

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
    cout<<111<<endl;
    srand(time(NULL));
    ros::init(argc,argv,"Lidar_node");
    Lidar_node node;
    node.process2();
    //ros::spin();

    return 0;
}
