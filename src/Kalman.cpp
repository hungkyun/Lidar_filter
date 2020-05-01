
#include "Kalman.h"

using namespace std;


class Tracker{
public:
	Tracker();
	bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
	void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices);
	void tracking_match1(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr predict_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_cloud);
	void tracking_match2(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr predict_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_cloud);
	void processPointcloud(const sensor_msgs::PointCloud2 &scan);
	void init_kalman(cv::KalmanFilter &kalman, const Target_ &tar_temp);
	void getClusterVertex(const std::vector<cv::Point> &cluster, Vertex &tmp);
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher predict_pub;
	ros::Publisher estimate_pub;
	vector<Target_> tar_list;
	KM ikm;
	bool init;
	double maxDis;
	int track_num;
	int frame;
	int min_age;
};

Tracker::Tracker()
{
	tar_list.clear();
	init = false;
	track_num = 0;
	maxDis = 5.0;
	frame = 0;
	min_age = -2;
	points_sub = nh.subscribe("all_points", 1028, &Tracker::processPointcloud, this);
	predict_pub = nh.advertise<sensor_msgs::PointCloud2>("/predict_points", 10);
	estimate_pub = nh.advertise<sensor_msgs::PointCloud2>("/estimated_points", 10);
}


void Tracker::processPointcloud(const sensor_msgs::PointCloud2 &scan)
{
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(scan,pcl_pc);
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromPCLPointCloud2(pcl_pc,*temp_cloud); // all points' data are stored in temp_cloud
	std::cout << "frame: " << frame++ << std::endl;
	std::cout << "point num: " << temp_cloud->points.size() << std::endl;
	std::vector<pcl::PointIndices> cluster_indices;
	clustering(temp_cloud,cluster_indices);

	pcl::PointCloud<pcl::PointXYZ>::Ptr predict_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < tar_list.size(); ++i)
	{
		tar_list[i].match = -1;
	}
	ikm.km_init(); // initialize km
	//tracking_match1(temp_cloud,cluster_indices,predict_cloud,estimated_cloud);
	tracking_match2(temp_cloud,cluster_indices,predict_cloud,estimated_cloud);

	sensor_msgs::PointCloud2 pub_predict;
	sensor_msgs::PointCloud2 pub_estimated;

	if(predict_cloud->points.size()!=0)
	{
		pcl::toROSMsg(*predict_cloud, pub_predict);
		pub_predict.header.frame_id = "velodyne";
		predict_pub.publish(pub_predict);
	}
	if(estimated_cloud->points.size()!=0)
	{
		pcl::toROSMsg(*estimated_cloud, pub_estimated);
		pub_estimated.header.frame_id = "velodyne";
		estimate_pub.publish(pub_estimated);
	}
	init = true;
	cout << endl;
}

//least square solution
bool Tracker::polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
	//number of key points
	int N = key_point.size();
 
	//construct matrix X
	cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) +
					std::pow(key_point[k].x, i + j);
			}
		}
	}
 
	//construct matrix Y
	cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) +
				std::pow(key_point[k].x, i) * key_point[k].y;
		}
	}
 
	A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	//get matrix A
	cv::solve(X, Y, A, cv::DECOMP_LU);
	return true;
}


void Tracker::getClusterVertex(const std::vector<cv::Point> &cluster, Vertex &tmp)
{
	float maxx(-9999),minx(9999),maxy(-9999),miny(9999); // upperVertex (maxx,maxy), lowerVertex (minx,miny)
	float y_max(0),y_min(0);
	float x_max(0),x_min(0);
	
	for (int i = 0; i < cluster.size(); ++i)
	{
		if(cluster[i].x > maxx)
		{
			maxx = cluster[i].x;
			y_max = cluster[i].y;
		}
		if(cluster[i].y > maxy)
		{
			maxy = cluster[i].y;
			x_max = cluster[i].x;
		}
		if(cluster[i].x < minx)
		{
			minx = cluster[i].x;
			y_min = cluster[i].y;
		}
		if(cluster[i].y < miny)
		{
			miny = cluster[i].y;	
			x_min = cluster[i].x;
		}
	}

	tmp.upper = cv::Point(maxx,maxy);
	tmp.lower = cv::Point(minx,miny);
	tmp.right_point = cv::Point(maxx,y_max);
	tmp.left_point = cv::Point(minx,y_min);
	tmp.top_point = cv::Point(x_max,maxy);
	tmp.low_point = cv::Point(x_min,miny);
	tmp.mid_point = cv::Point((minx+maxx)/2,(miny+maxy)/2);

	if((maxx-minx)>(maxy-miny))
	{
		tmp.flag = 0;
		tmp.longth = sqrt(pow(tmp.right_point.x-tmp.left_point.x,2)+pow(tmp.right_point.y-tmp.left_point.y,2));
		tmp.begin = tmp.left_point;
		tmp.end = tmp.right_point;
	}
	else
	{
		tmp.flag = 1;
		tmp.longth = sqrt(pow(tmp.top_point.x-tmp.low_point.x,2)+pow(tmp.top_point.y-tmp.low_point.y,2));
		tmp.begin = tmp.low_point;
		tmp.end = tmp.top_point;
	}
}

void Tracker::clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices)
{
	
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud (in_cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (5);
	ec.setMinClusterSize (10);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (in_cloud);
	ec.extract (cluster_indices);
}

void Tracker::init_kalman(cv::KalmanFilter &kalman, const Target_ &tar_temp)
{
	cv::KalmanFilter new_kf(6,3,0,CV_64FC1);
	new_kf.transitionMatrix = (cv::Mat_<double>(6,6) <<1,0,0,1,0,0,
			0,1,0,0,1,0,
			0,0,1,0,0,1,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,0,0,0,1);
	new_kf.statePost.at<double>(0) = tar_temp.param.at<double>(0, 0);
	new_kf.statePost.at<double>(1) = tar_temp.param.at<double>(1, 0);
	new_kf.statePost.at<double>(2) = tar_temp.param.at<double>(2, 0);
	new_kf.statePost.at<double>(3) = 0.0;
	new_kf.statePost.at<double>(4) = 0.0;
	new_kf.statePost.at<double>(5) = 0.0;
	cv::setIdentity(new_kf.measurementMatrix);
	cv::setIdentity(new_kf.processNoiseCov,cv::Scalar::all(0.2*0.2));
	cv::setIdentity(new_kf.errorCovPost,cv::Scalar::all(0.1));
	cv::setIdentity(new_kf.measurementNoiseCov,cv::Scalar::all(0.1));
	kalman = new_kf;
}

void Tracker::tracking_match1(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr predict_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_cloud)
{
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i)
	{
		std::cout << "new target" << std::endl;
		std::vector<cv::Point> points_2d;
		Target_ tar_temp;
		double meanx(0.0), meany(0.0);
		int count(1);
		for (std::vector<int>::const_iterator pit = i->indices.begin (); pit != i->indices.end (); ++pit)
		{
			points_2d.push_back(cv::Point((in_cloud->points[*pit]).x,(in_cloud->points[*pit]).y));
			meanx += (in_cloud->points[*pit]).x;
			meany += (in_cloud->points[*pit]).y;
			count++;
		}
		tar_temp.target_point = cv::Point(meanx/count,meany/count);
		getClusterVertex(points_2d,tar_temp.vertex);

		polynomial_curve_fit(points_2d, 2, tar_temp.param);

		//kalmanFilter
		if(!init)
		{
			tar_temp.trace_ID = track_num;
			track_num++;
			init_kalman(tar_temp.kalman,tar_temp);
			tar_list.push_back(tar_temp);
		}
		else
		{
			double dis_judge(0.0);
			int match_id = -1;
			for (int j = 0; j < tar_list.size(); ++j)
			{
				dis_judge = pow((tar_list[j].target_point.x-tar_temp.target_point.x),2) + pow((tar_list[j].target_point.y-tar_temp.target_point.y),2);
				if(dis_judge<=80 && tar_list[j].match==0)
				{
					tar_temp.trace_ID = tar_list[j].trace_ID;
					tar_list[j].match = 1;
					match_id = j;
				}
			}

			if(match_id==-1)
			{
				tar_temp.trace_ID = track_num;
				track_num++;
				init_kalman(tar_temp.kalman,tar_temp);
				tar_list.push_back(tar_temp);
				cout << "no match" << endl;
			}
			else
			{
				cv::setIdentity(tar_list[match_id].kalman.measurementNoiseCov,cv::Scalar::all(0.1));
				cv::Mat predict = tar_list[match_id].kalman.predict();
				cv::Mat measure(3,1,CV_64FC1);
				measure.at<double>(0) = tar_temp.param.at<double>(0,0);
				measure.at<double>(1) = tar_temp.param.at<double>(1,0);
				measure.at<double>(2) = tar_temp.param.at<double>(2,0);

				cv::Mat estimated = tar_list[match_id].kalman.correct(measure);
				tar_temp.param = estimated;
				tar_temp.pred = predict;
				tar_temp.kalman = tar_list[match_id].kalman;

				double xbengin,xend;
				if(tar_temp.vertex.begin.x > tar_temp.vertex.end.x)
				{
					xbengin = tar_temp.vertex.end.x;
					xend = tar_temp.vertex.begin.x;
				}
				else
				{
					xbengin = tar_temp.vertex.begin.x;
					xend = tar_temp.vertex.end.x;
				}
				for (double x = xbengin; x < xend;)
				{
					double y = tar_temp.pred.at<double>(0, 0) + tar_temp.pred.at<double>(1, 0) * x +
							   tar_temp.pred.at<double>(2, 0) * x * x;
					predict_cloud->points.push_back(pcl::PointXYZ(float(x), float(y), 0));

					double y1 = tar_temp.param.at<double>(0, 0) + tar_temp.param.at<double>(1, 0) * x +
								tar_temp.param.at<double>(2, 0) * x * x;
					estimated_cloud->points.push_back(pcl::PointXYZ(float(x), float(y1), 0));

					x += 0.01;
				}
				cout << "predict" << endl;
				tar_list[match_id] = tar_temp; //update
			}
		}
	}
}

void Tracker::tracking_match2(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr predict_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_cloud){

	vector<vector<cv::Point>> points;
	vector<Target_> tar_now;
	vector<double> disv;
	points.clear();
	tar_now.clear();
	disv.clear();
	int count=0;
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i)
	{
		std::vector<cv::Point> points_2d;
		Target_ tar_temp;
		double meanx(0.0), meany(0.0);
		for (std::vector<int>::const_iterator pit = i->indices.begin (); pit != i->indices.end (); ++pit)
		{
			points_2d.push_back(cv::Point((in_cloud->points[*pit]).x,(in_cloud->points[*pit]).y));
			meanx += points_2d.back().x;
			meany += points_2d.back().y;
		}
		cout << "cluster. " << count++ << " " << points_2d.size() << endl;
		tar_temp.point_num = points_2d.size();
		tar_temp.target_point = cv::Point(meanx/(points_2d.size()*1.0),meany/(points_2d.size()*1.0));
		cout << "tar_point: " << tar_temp.target_point.x << " " << tar_temp.target_point.y << endl;
		getClusterVertex(points_2d,tar_temp.vertex);
		tar_temp.longth = tar_temp.vertex.longth;
		polynomial_curve_fit(points_2d, 2, tar_temp.param);

		//points.push_back(points_2d);
		tar_now.push_back(tar_temp);
	}

	// KM match
	cout << "tar_now size: " << tar_now.size() << endl;
	cout << "tar_list size: " << tar_list.size() << endl;
	if(init)
	{
		cout << "--------------------" << endl;
		//weight=n1/(d*n2), w1=last frame point num, w2=this frame point num
		for (int i = 0; i < tar_now.size(); ++i)
		{
			cout << "now target: " << tar_now[i].target_point.x << " " << tar_now[i].target_point.y << " num: " << tar_now[i].point_num << endl;

			for (int j = 0; j < tar_list.size(); ++j)
			{
				cout << "last target: " << tar_list[j].target_point.x << " " << tar_list[j].target_point.y << " num: " << tar_list[j].point_num << " age: " << tar_list[j].age<< endl;
				double dis = sqrt(pow(tar_list[j].target_point.x-tar_now[i].target_point.x,2)+pow(tar_list[j].target_point.y-tar_now[i].target_point.y,2));
				disv.push_back(dis);
				//ikm.mmp[i][j] = 1000 * tar_now[i].point_num / (tar_list[j].point_num * dis * 1.0 + 1.0);
				ikm.mmp[i][j] =  1000.0 / (sqrt(abs(tar_now[i].point_num - tar_list[j].point_num)) + dis*dis*dis + 1.0);
				//ikm.mmp[i][j] = 1000 * (1 - dis/maxDis) / sqrt(abs(tar_now[i].point_num-tar_list[j].point_num));
				//ikm.mmp[i][j] = 1000.0 / ((dis+1) * sqrt(abs(tar_now[i].point_num-tar_list[j].point_num)) * (1 - tar_list[j].age));
				cout<< i << " " << j << " " << "distance: "<< dis <<endl;
				cout << "weight: " << ikm.mmp[i][j] << endl;
			}
		}
		cout << "--------------------" << endl;
		//if n1 != n2, full mmp[][] with -1, optional step
		if(tar_list.size()!=tar_now.size())
		{
			// expand row
			if(tar_list.size()>tar_now.size())
			{
				for (int i = tar_now.size(); i < tar_list.size(); ++i)
				{
					for (int j = 0; j < tar_list.size(); ++j) {
						ikm.mmp[i][j] = -5.0;
					}
				}
				ikm.num = tar_list.size();
			}
			// expand column
			else
			{
				for (int i = 0; i < tar_now.size(); ++i) {
					for (int j = tar_list.size(); j < tar_now.size(); ++j) {
						ikm.mmp[i][j] = -5.0;
					}
				}
				ikm.num = tar_now.size();
			}
		}
		else{
			ikm.num = tar_now.size();
		}

		cout << "++++++++++++++++++++++++" << endl;
		for (int l = 0; l < ikm.num; ++l) {
			for (int i = 0; i < ikm.num; ++i) {
				cout << ikm.mmp[l][i] << " ";
			}
			cout << endl;
		}

		double ans = ikm.ikm_match();
		cout << "Ex. " << ans << endl;
		for (int m = 0; m < ikm.num; ++m) {
			cout << ikm.imatch[m] << " ";
		}
		cout << "++++++++++++++++++++++++" << endl;
		cout << endl;
		for (int k = 0; k < tar_now.size(); ++k) {
			if(ikm.mmp[k][ikm.imatch[k]] < 0)
				tar_now[k].match = -1;
			else{
				int index = k*tar_list.size()+ikm.imatch[k];
				if(disv[index] > maxDis){
					tar_now[k].match = -1;
				}
				else{
					tar_list[ikm.imatch[k]].match = k;
					tar_now[k].match = ikm.imatch[k];
				}
			}
		}
	}

	// kalman tracker
	for (int k = 0; k < tar_now.size(); ++k)
	{
		if(!init)
		{
			init_kalman(tar_now[k].kalman,tar_now[k]);
			tar_list.push_back(tar_now[k]);
		}
		else
		{
			// no match
			if(tar_now[k].match < 0)
			{
				init_kalman(tar_now[k].kalman,tar_now[k]);
				tar_list.push_back(tar_now[k]);
			}
			else
			{
				int match_id = tar_now[k].match;
				cv::setIdentity(tar_list[match_id].kalman.measurementNoiseCov,cv::Scalar::all(0.1));
				cv::Mat predict = tar_list[match_id].kalman.predict();
				cv::Mat measure(3,1,CV_64FC1);
				measure.at<double>(0) = tar_now[k].param.at<double>(0,0);
				measure.at<double>(1) = tar_now[k].param.at<double>(1,0);
				measure.at<double>(2) = tar_now[k].param.at<double>(2,0);

				cv::Mat estimated = tar_list[match_id].kalman.correct(measure);
				tar_now[k].param = estimated;
				tar_now[k].pred = predict;
				tar_now[k].kalman = tar_list[match_id].kalman;

				if(tar_now[k].longth < (0.85*tar_list[match_id].longth)){
					tar_now[k].longth = 0.85*tar_list[match_id].longth;
				}
				
				double xbengin,xend;
				if(tar_now[k].vertex.begin.x > tar_now[k].vertex.end.x){
					xbengin = tar_now[k].vertex.end.x;
					xend = tar_now[k].vertex.begin.x;
				}
				else{
					xbengin = tar_now[k].vertex.begin.x;
					xend = tar_now[k].vertex.end.x;
				}
				for (double x = xbengin; x < xend;) {
					double y = tar_now[k].pred.at<double>(0, 0) + tar_now[k].pred.at<double>(1, 0) * x +
							tar_now[k].pred.at<double>(2, 0) * x * x;
					predict_cloud->points.push_back(pcl::PointXYZ(float(x), float(y), 0));

					double y1 = tar_now[k].param.at<double>(0, 0) + tar_now[k].param.at<double>(1, 0) * x +
							tar_now[k].param.at<double>(2, 0) * x * x;
					estimated_cloud->points.push_back(pcl::PointXYZ(float(x), float(y1), 0));

					x += 0.01;
				}
				cout << "predict" << endl;
				tar_now[k].age = tar_list[match_id].age + 1;
				tar_list[match_id] = tar_now[k]; //update
			}
		}
	}

	// delete non-match objects
	if(init){
		for (int i = 0; i < tar_list.size(); ++i) {
			if(tar_list[i].match < 0){
				tar_list[i].age--;
			}
			if(tar_list[i].age < min_age){
				tar_list.erase(tar_list.begin()+i);
			}
		}
	}
}


int main(int argc, char **argv)
{
	std::cout << "kalman" << std::endl;
    ros::init(argc, argv, "kalman_node");
    Tracker tracker;
    ros::spin();

	/*
	vector<int> a;
	a.push_back(1);
	a.push_back(2);
	a.push_back(3);
	a.erase(a.begin());
	cout << a[0] << " " << a[1] << endl;
    */
    // test KM algorithm


	KM ikm;
	ikm.km_init();
	ikm.num = 5;
	ikm.mmp[0][0] = 99.9171;
	ikm.mmp[0][1] = 0.879795;
	ikm.mmp[0][2] = 22.6896;
	ikm.mmp[0][3] = 0.009008;
	ikm.mmp[0][4] = 22.726;
	ikm.mmp[1][0] = 0.168707;
	ikm.mmp[1][1] = 100.284;
	ikm.mmp[1][2] = 13.2233;
	ikm.mmp[1][3] = 105800;
	ikm.mmp[1][4] = 14.4516;
	ikm.mmp[2][0] = -500;
	ikm.mmp[2][1] = -500;
	ikm.mmp[2][2] = -500;
	ikm.mmp[2][3] = -500;
	ikm.mmp[2][4] = -500;
	ikm.mmp[3][0] = -500;
	ikm.mmp[3][1] = -500;
	ikm.mmp[3][2] = -500;
	ikm.mmp[3][3] = -500;
	ikm.mmp[3][4] = -500;
	cout << ikm.ikm_match() << endl;
	for (int i = 0; i < ikm.num; ++i) {
		cout << ikm.imatch[i] << endl;
	}

    return 0;
}

