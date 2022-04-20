#undef UNICODE
#undef _UNICODE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>// 球半径滤波器
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波器 


typedef pcl::PointCloud<pcl::PointXYZI>  Cloud;

int
main(int argc, char** argv)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_filtered_ptr(new pcl::PCLPointCloud2);
	Cloud::Ptr cloud_filtered_1(new Cloud);
	//* the data should be available in cloud
	PCL_INFO("Starting processing \n");

	time_t start_time, end_time;
	time(&start_time);

	// 填入点云数据
	pcl::PCDReader reader;
	// 把路径改为自己存放文件的路径
	reader.read("lvbo.pcd", *cloud); // 记住要事先下载这个数据集！
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	//// 创建滤波器对象
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.1f, 0.1f, 0.1f);
	sor.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	/*pcl::PCDWriter writer;
	writer.write("2f1dm.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);*/

	time(&end_time);
	PCL_INFO("Time taken to complete VoxelGrid: %.2f seconds\n", difftime(end_time, start_time));

	/*// 创建滤波器　
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> Radius;
	// 建立滤波器
	Radius.setInputCloud(cloud_filtered_ptr);
	Radius.setRadiusSearch(0.2);//半径为　0.8ｍ
	Radius.setMinNeighborsInRadius(10);//半径内最少需要　２个点
	// 执行滤波
	Radius.filter(*cloud_filtered_1);

	pcl::PCDWriter writer;
	writer.writeASCII("radius.pcd", *cloud_filtered_1, 8);*/
	//pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sta;//创建滤波器对象
	//sta.setInputCloud(cloud);		    //设置待滤波的点云
	//sta.setMeanK(100);	     			    //设置在进行统计时考虑查询点临近点数
	//sta.setStddevMulThresh(0.5);	   		    //设置判断是否为离群点的阀值
	//sta.filter(*cloud_filtered_ptr); 		    //存储内点

	//pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_filtered_1);

	pcl::PCDWriter writer; 
	//writer.writeASCII("radius.pcd", *cloud_filtered_1, 8);
	writer.write("table_scene_lms400_inliers.pcd", *cloud_filtered_ptr, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_filtered_1);

	// 程序可视化
	pcl::visualization::CloudViewer viewer("pcd viewer");// 显示窗口的名字
	viewer.showCloud(cloud_filtered_1);
	while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}
	return (0);
}
