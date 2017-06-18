#include"visual.h"


void cloud_visual(PointCloud<PointXYZ>& pointcloud)
{
	PointCloud<pcl::PointXYZ>::Ptr cloudPointer;
	cloudPointer = pointcloud.makeShared();
	visualization::PCLVisualizer viewer("viewer");
	viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
	viewer.addCoordinateSystem(0.3);
	viewer.addPointCloud(cloudPointer);
	
	while (!viewer.wasStopped())
		viewer.spinOnce(100);
}

Eigen::Matrix4f estimation(const PointCloud<pcl::PointXYZ>& cloud_src, const vector<int>& index1, const PointCloud< pcl::PointXYZ>& cloud_tgt, const vector<int> index2)
{
	Eigen::Matrix4f transformation_matrix;
	registration::TransformationEstimationSVD<PointXYZ,PointXYZ,float> trans;
	trans.estimateRigidTransformation(cloud_src, index1, cloud_tgt, index2, transformation_matrix);
	return transformation_matrix;
}

bool convert_cloud(const vector<pcl::PointXYZ >& keypoint, PointCloud< pcl::PointXYZ>& cloud)
{
	cloud.width = keypoint.size();
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	vector<PointXYZ>::const_iterator it;//³£Á¿µü´úÆ÷
	int i = 0;
	for (it = keypoint.begin(); it != keypoint.end() && i<keypoint.size(); it++, i++)
	{
		const PointXYZ point = (*it);
		cloud.points[i].x = point.x;
		cloud.points[i].y = point.y;
		cloud.points[i].z = point.z;
	}
	return true;
}

bool get_index(vector<int>& index, int size)
{
	for (int i = 0; i < size; i++)
		index.push_back(i);
	return true;
}


double validation(PointCloud<PointXYZ>& cloud1,PointCloud<PointXYZ>& cloud2, Eigen::Matrix4f & trans_mat)
{
	registration::TransformationValidationEuclidean<PointXYZ, PointXYZ, float> valid;
	PointCloud<pcl::PointXYZ>::Ptr cloudPointer1;
	PointCloud<pcl::PointXYZ>::Ptr cloudPointer2;
	cloudPointer1 = cloud1.makeShared();
	cloudPointer2 = cloud2.makeShared();
	return valid.validateTransformation(cloudPointer2, cloudPointer2, trans_mat);
}