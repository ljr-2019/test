#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/common/io.h>
#include <iostream>
#include <pcl/keypoints/sift_keypoint.h>//关键点检测
using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

//点云可视化
void visualize_pcd(PointCloud::Ptr pcd_src,
    PointCloud::Ptr pcd_tgt)
    //PointCloud::Ptr pcd_final)
{


    pcl::visualization::PCLVisualizer viewer("registration Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(pcd_src, src_h, "source cloud");
    viewer.addPointCloud(pcd_tgt, tgt_h, "tgt cloud");
    //viewer.addPointCloud(pcd_final, final_h, "final cloud");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "tgt cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
       // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    //pcl中sift特征需要返回强度信息，改为如下:
}
namespace pcl
{
    template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
        inline float
            operator () (const PointXYZ& p) const
        {
            return p.z;
        }
    };
}



int
main(int argc, char** argv)
{
    //加载点云文件
    PointCloud::Ptr cloud_src_o(new PointCloud);//原点云，待配准
    pcl::io::loadPCDFile("11.pcd", *cloud_src_o);
    double resolution = computeCloudResolution(cloud_src_o);
    cout << "原始点云数量：" << cloud_src_o->size() << endl;
    cout << "原始点云resolution：" << resolution << endl;
    //PointCloud::Ptr cloud_tgt_o(new PointCloud);//目标点云
    //pcl::io::loadPCDFile("22.pcd", *cloud_tgt_o);
    //cout << "原始点云数量：" << cloud_tgt_o->size() << endl;
    //cout << "原始点云resolution：" << resolution << endl;
    clock_t start = clock();
    //去除NAN点
    std::vector<int> indices_src; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(*cloud_src_o, *cloud_src_o, indices_src);
    std::cout << "remove *cloud_src_o nan" << endl;

  /* std::vector<int> indices_tgt;
    pcl::removeNaNFromPointCloud(*cloud_tgt_o, *cloud_tgt_o, indices_tgt);
    std::cout << "remove *cloud_tgt_o nan" << endl;*/


    //设定参数值
    const float min_scale = 0.002f; //the standard deviation of the smallest scale in the scale space
    const int n_octaves = 3;//尺度空间层数,小、关键点多
    const int n_scales_per_octave = 3;//the number of scales to compute within each octave
    const float min_contrast = 0.00025f;//根据点云，设置大小，越小关键点越多

    //sift关键点检测
    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale > sift_src;
    pcl::PointCloud<pcl::PointWithScale> result_src;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
    sift_src.setSearchMethod(tree_src);
    sift_src.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift_src.setMinimumContrast(min_contrast);
    sift_src.setInputCloud(cloud_src_o);
    sift_src.compute(result_src);

   /* pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale > sift_tgt;
    pcl::PointCloud<pcl::PointWithScale> result_tgt;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointXYZ>());
    sift_src.setSearchMethod(tree_tgt);
    sift_src.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift_src.setMinimumContrast(min_contrast);
    sift_src.setInputCloud(cloud_tgt_o);
    sift_src.compute(result_tgt);*/
    clock_t end = clock();
    cout << "sift关键点提取时间" << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << "sift关键点数量" << result_src.size() << endl;

    /*cout << "sift关键点提取时间" << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << "sift关键点数量" << result_tgt.size() << endl;*/
    PointCloud::Ptr cloud_src(new PointCloud);
    pcl::copyPointCloud(result_src, *cloud_src);
    std::cout << "down size *cloud_src_o from " << cloud_src_o->size() << "to" << cloud_src->size() << endl;
    /* pcl::io::savePCDFileASCII("211.pcd", *cloud_src);*/

   /* PointCloud::Ptr cloud_tgt(new PointCloud);
    pcl::copyPointCloud(result_tgt, *cloud_tgt);
    std::cout << "down size *cloud_tgt_o from " << cloud_tgt_o->size() << "to" << cloud_tgt->size() << endl;
   pcl::io::savePCDFileASCII("222.pcd", *cloud_tgt);*/

    //可视化
    visualize_pcd(cloud_src_o, cloud_src);
    /*visualize_pcd(cloud_tgt_o, cloud_tgt);*/
    return (0);
}
double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}
