#undef UNICODE
#undef _UNICODE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>
#include <pcl/filters/radius_outlier_removal.h>// ��뾶�˲���
#include <pcl/filters/statistical_outlier_removal.h>//ͳ���˲��� 


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

	// �����������
	pcl::PCDReader reader;
	// ��·����Ϊ�Լ�����ļ���·��
	reader.read("lvbo.pcd", *cloud); // ��סҪ��������������ݼ���
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";
	//// �����˲�������
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

	/*// �����˲�����
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> Radius;
	// �����˲���
	Radius.setInputCloud(cloud_filtered_ptr);
	Radius.setRadiusSearch(0.2);//�뾶Ϊ��0.8��
	Radius.setMinNeighborsInRadius(10);//�뾶��������Ҫ��������
	// ִ���˲�
	Radius.filter(*cloud_filtered_1);

	pcl::PCDWriter writer;
	writer.writeASCII("radius.pcd", *cloud_filtered_1, 8);*/
	//pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sta;//�����˲�������
	//sta.setInputCloud(cloud);		    //���ô��˲��ĵ���
	//sta.setMeanK(100);	     			    //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
	//sta.setStddevMulThresh(0.5);	   		    //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
	//sta.filter(*cloud_filtered_ptr); 		    //�洢�ڵ�

	//pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_filtered_1);

	pcl::PCDWriter writer; 
	//writer.writeASCII("radius.pcd", *cloud_filtered_1, 8);
	writer.write("table_scene_lms400_inliers.pcd", *cloud_filtered_ptr, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_filtered_1);

	// ������ӻ�
	pcl::visualization::CloudViewer viewer("pcd viewer");// ��ʾ���ڵ�����
	viewer.showCloud(cloud_filtered_1);
	while (!viewer.wasStopped())
	{
		// Do nothing but wait.
	}
	return (0);
}
