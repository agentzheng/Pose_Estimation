#pragma once
#include"all.h"

void cloud_visual(PointCloud<PointXYZ>& pointcloud);
Eigen::Matrix4f estimation(const PointCloud< pcl::PointXYZ>& cloud_src, const vector<int>& index1, const PointCloud< pcl::PointXYZ>& cloud_tgt, const vector<int> index2);
bool convert_cloud(const vector<pcl::PointXYZ >& keypoint, PointCloud< pcl::PointXYZ>& cloud);
bool get_index(vector<int>& index, int size);
double validation(PointCloud<PointXYZ>& cloud1, PointCloud<PointXYZ>& cloud2, Eigen::Matrix4f & trans_mat);