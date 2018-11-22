#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


int  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // reading parameters
  
  cv::FileStorage fs("image3D.xml", cv::FileStorage::READ);
  if (!fs.isOpened() )
     {
       std::cout << "Failed to open stereoParams.xml" << std::endl;
       return 1;
     }  
  cv::Mat pos3D;
  fs["image3D"] >>  pos3D;

  fs.release();

  cv::Size matSize = pos3D.size();

  std::cout << "mat: " << matSize << std::endl;
  (*cloud).width    = matSize.width;
  (*cloud).height   = matSize.height;
  (*cloud).is_dense = false;
  (*cloud).points.resize ((*cloud).width * (*cloud).height);
 
  int p=0;
  for (int i=0;i< (*cloud).height; i++)
    for (int j=0;j< (*cloud).width; j++){
      (*cloud).points[p].x = pos3D.at<cv::Vec3f>(i,j)[0];
      (*cloud).points[p].y = pos3D.at<cv::Vec3f>(i,j)[1];
      (*cloud).points[p].z = pos3D.at<cv::Vec3f>(i,j)[2];
      p++;
    }
 
 
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  
  while (!viewer.wasStopped())
    {
    }
 
  return (0);
}