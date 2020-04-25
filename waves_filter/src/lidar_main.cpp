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
	void process1();
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

struct gridMap{
	bool isNoise;
	int num;
	int x,y;
	float maxH,minH;
	float diffH;
	float dis;
	gridMap():isNoise(true),num(0),maxH(-999),minH(999),diffH(0),dis(0){}
};

struct Vertex{
	float upper_left[2];
	float lower_right[2];
	Vertex(){
		upper_left[0] = 0; // maxx
		upper_left[1] = 0; // maxy
		lower_right[0] = 0;// minx
		lower_right[1] = 0;// miny
	}
};

Vertex getClusterVertex(pcl::PointCloud<pcl::PointXYZ> &cluster, Vertex &tmp)
{
	float maxx(-9999),minx(9999),maxy(-9999),miny(9999); // upperVertex (maxx,maxy), lowerVertex (minx,miny)
	for (int i = 0; i < cluster.size(); ++i) {
		if(cluster[i].x > maxx)
			maxx = cluster[i].x;
		if(cluster[i].y > maxy)
			maxy = cluster[i].y;
		if(cluster[i].x < minx)
			minx = cluster[i].x;
		if(cluster[i].y < miny)
			miny = cluster[i].y;
	}
	tmp.upper_left[0] = maxx;
	tmp.upper_left[1] = maxy;
	tmp.lower_right[0] = minx;
	tmp.lower_right[1] = miny;
	return tmp;
}


float minDis = 5;
float maxDis = 40;
float radius = 3;

void kdtreeSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ> &pcloud2, vector<Vertex> &clusterInfo)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);
	for (int n = 0; n < pcloud2.size(); ++n) {
		vector<int> pointRadiusSearch;
		vector<float> pointRadiusSquareDistance;
		int value = kdtree->radiusSearch(pcloud2[n], radius, pointRadiusSearch, pointRadiusSquareDistance);
		if(value > 2){
			Vertex tmp;
			pcl::PointCloud<pcl::PointXYZ> points;
			for (int i = 0; i < pointRadiusSearch.size(); ++i) {
				points.push_back(pcloud2[pointRadiusSearch[i]]);
			}
			getClusterVertex(points, tmp);
			clusterInfo.push_back(tmp);
		}
	}
}

void cmp(int i, int j, int &xmin, int &ymin, int &xmax, int &ymax)
{
	if(i < xmin)
		xmin = i;
	if(i > xmax)
		xmax = i;
	if(j < ymin)
		ymin = j;
	if(j > ymax)
		ymax = j;
}

void regionGrow(gridMap **map, vector<Vertex> &clusterInfo, float grid_size, float minx, float maxx, float miny, float maxy)
{
	int grow_direction[8][2]={{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0}};
	int cols = (maxx - minx) / grid_size + 1;
	int rows = (maxy - miny) / grid_size + 1;
	for (int i = 1; i < rows-1; ++i)
	{
		for (int j = 1; j < cols-1; ++j)
		{
			int xmin=999,xmax=-999,ymin=999,ymax=-999;
			vector<gridMap> seed;
			seed.clear();
			if(map[i][j].isNoise == false)
			{
				seed.push_back(map[i][j]);
				cmp(i,j,xmin,ymin,xmax,ymax);
				int m,n;
				while(!seed.empty())
				{
					gridMap current = seed.back();
					map[current.x][current.y].isNoise = true;
					seed.pop_back();
					for (int k = 0; k < 8; ++k) {
						m = current.x + grow_direction[k][0];
						n = current.y + grow_direction[k][1];
						if(m < 0 || n < 0 || m > (rows-1) || n > (cols-1))
							continue;
						if(map[m][n].isNoise == false)
						{
							seed.push_back(map[m][n]);
							cmp(m,n,xmin,ymin,xmax,ymax);
						}
					}
				}

				Vertex tmp;
				tmp.upper_left[0] = xmin+minx;
				tmp.upper_left[1] = ymin+miny;
				tmp.lower_right[0] = xmax+minx;
				tmp.lower_right[1] = ymax+miny;
				clusterInfo.push_back(tmp);
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZ> gridMap_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float grid_size, float minx, float maxx, float miny, float maxy, vector<Vertex> &clusterInfo)
{
	float minDiffH = 0.20; // minimum height difference
	int minNum = 3;
	// initial
	int cols = (maxx - minx) / grid_size + 1;
	int rows = (maxy - miny) / grid_size + 1;
	gridMap **map = new gridMap *[rows];
	for (int i = 0; i < rows; ++i) {
		map[i] = new gridMap[cols];
	}

	// filling map and filtering
	for (int k = 0; k < cloud->points.size(); ++k) {
		auto pc = cloud->points[k];
		float dis = sqrt((pc.x*pc.x)+(pc.y*pc.y));
		if(dis < minDis)
			continue;
		int i = (pc.y - miny) / grid_size;
		int j = (pc.x - minx) / grid_size;
		map[i][j].dis = dis;
		map[i][j].num += 1;
		if(pc.z > map[i][j].maxH)
			map[i][j].maxH = pc.z;
		if(pc.z < map[i][j].minH)
			map[i][j].minH = pc.z;
	}

	int direction[4][2] = {{-1,0},{1,0},{0,1},{0,-1}};
	pcl::PointCloud<pcl::PointXYZ> pcloud,pcloud2;
	for (int l = 0; l < rows; ++l) {
		for (int m = 0; m < cols; ++m) {
			map[l][m].x = l;
			map[l][m].y = m;
			map[l][m].diffH = map[l][m].maxH - map[l][m].minH;
			// filter out outliers
			if((map[l][m].dis < maxDis && map[l][m].diffH < minDiffH) || map[l][m].num < minNum){
				map[l][m].isNoise = true;
			}
			else{
				map[l][m].isNoise = false;
				for (int i = 0; i < 4; ++i) { // extend the map boundary to improve robustness
					int xx = l + direction[i][0], yy = m + direction[i][1];
					if(xx < 0 || yy < 0 || xx > (rows-1) || yy > (cols-1))
						continue;
					map[xx][yy].isNoise = false;
				}

				// output pointcloud, filter results
				pcl::PointXYZ p1(m+minx,l+miny,0),p2(m+minx,l+miny+1,0),p3(m+minx+1,l+miny,0),p4(m+minx+1,l+miny+1,0);
				pcloud.push_back(p1);
				pcloud.push_back(p2);
				pcloud.push_back(p3);
				pcloud.push_back(p4);
			}
		}
	}

	regionGrow(map,clusterInfo,grid_size,minx,maxx,miny,maxy);

	cout << "cluster num: " << clusterInfo.size() << endl;
	for (int i = 0; i < clusterInfo.size(); ++i) {
		cout << "No." << i << endl;
		cout << "upper: " << clusterInfo[i].upper_left[0] << " " << clusterInfo[i].upper_left[1] << endl;
		cout << "lower: " << clusterInfo[i].lower_right[0] << " " << clusterInfo[i].lower_right[1] << endl;
		cout << "-----------------" << endl;
	}

	delete[] map;
	return pcloud;
}

void Lidar_node::process1(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i < 3; i++){
		ifstream in;
		string str = "/home/yxt/conference/yunzhou/waves_filter/data/";
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


		float maxx=-9999, maxy=-9999, minx=9999, miny=9999, grid_size = 1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
		for (int j = 0; j < cloud->points.size(); ++j) {
			if(cloud->points[j].x > maxx)
				maxx = cloud->points[j].x;
			if(cloud->points[j].x < minx)
				minx = cloud->points[j].x;
			if(cloud->points[j].y > maxy)
				maxy = cloud->points[j].y;
			if(cloud->points[j].y < miny)
				miny = cloud->points[j].y;
		}

		pcl::PointCloud<pcl::PointXYZ> result;
		vector<Vertex> clusterInfo; // cluster result
		result = gridMap_filter(cloud,grid_size,minx,maxx,miny,maxy,clusterInfo);


		cout << str << endl;
		sensor_msgs::PointCloud2 pp;
		pcl::toROSMsg(*cloud,pp);
		pp.header.frame_id = "pandar";
		pub.publish(pp);

		sensor_msgs::PointCloud2 fresult;
		pcl::toROSMsg(result,fresult);
		fresult.header.frame_id = "pandar";
		fpub.publish(fresult);
		sleep(2);
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
	main_int8();
	Lidar_node node;
	node.process1(); // filtering and clustering
    return 0;
}
