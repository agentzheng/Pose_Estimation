#include"visual.h"


#define POINT_NUMBER 5000

int main()
{
	PointCloud<PointXYZ> cloud;
	PointCloud<PointXYZ> cloud2;
	cloud.width = POINT_NUMBER;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud2.width = POINT_NUMBER;
	cloud2.height = 1;
	cloud2.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	cloud2.points.resize(cloud2.width*cloud2.height);

	for (int i = 0; i < POINT_NUMBER; i++)
	{
		cloud.points[i].x = 10;
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud2.points[i].x = 60;
		cloud2.points[i].y = cloud.points[i].y;
		cloud2.points[i].z = cloud.points[i].z;
	}

	// 1. �������

	vector<int> index;
	get_index(index, cloud.size() / 10);
	cout << "estimating" << endl;
	Eigen::Matrix4f trans_mat = estimation(cloud, index, cloud2, index);
	cout << trans_mat << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudIn(cloud.makeShared());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudIn2(cloud2.makeShared());

	pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, trans_mat);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_final->setBackgroundColor(0, 0, 0);     //���ñ���ɫΪ��ɫ
												   //��Դ������ɫ����ɫ�������ӻ�
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> In_color(pPointCloudIn, 255, 0, 0);
	//viewer_final->addPointCloud<pcl::PointXYZ>(pPointCloudIn, In_color, "origin");
	//viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "origin cloud");
	//��Ŀ�������ɫ����ɫ�������ӻ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>In2_color(pPointCloudIn2, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ>(pPointCloudIn2, In2_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	//��ת�����Դ����(��ɫ)�����ӻ�
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>out_color(pPointCloudOut, 0, 0, 255);
	viewer_final->addPointCloud<pcl::PointXYZ>(pPointCloudOut, out_color, "result cloud");
	viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "result cloud");
	// �������ӻ�
	viewer_final->addCoordinateSystem(1.0);      //��ʾxyzָʾ��
	viewer_final->initCameraParameters();         //��ʼ������ͷ������
												  //�ȴ�ֱ�����ӻ����ڹرա�
	while (!viewer_final->wasStopped())
	{
		viewer_final->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	//cout << a << endl;
	system("pause");
}