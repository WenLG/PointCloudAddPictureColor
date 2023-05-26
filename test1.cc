//./kitti-color ./kitti/1.pcd ./000001.png
#include <iostream>  
#include <boost/thread/thread.hpp>  
#include <pcl/common/common_headers.h>  
#include <pcl/common/common_headers.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/console/parse.h>  

#include "opencv2/highgui.hpp"
using namespace cv;
using namespace std;

//RGB colour visualisation example  
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    //创建3D窗口并添加点云   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

// -----Main-----  
int main(int argc, char** argv)
{
	// 解析命令行参数  
	if (argc != 3)
	{
			cout<<"Usage : xx yy.pcd zz.png"<<endl;
			//return 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile (argv[1], *point_cloud_ptr);
	
	Mat img = imread(argv[2]);
	if(img.channels() != 3){
		cout <<"RGB pics needed." <<endl;
		return 0;
	}
	cout <<"Pic loaded." <<endl;
	int rows = img.rows;
	int cols = img.cols;
	unsigned char red,green,blue;
	float p_u,p_v,p_w;//pics_uv1;(u for cols, v for lines!!!)
	float c_x,c_y,c_z,c_i;//clouds_xyz、intensity;
	
	Mat P2 = (Mat_<float>(3,4) << 7.215377000000e+02,0.000000000000e+00,6.095593000000e+02,4.485728000000e+01,0.000000000000e+00,7.215377000000e+02,1.728540000000e+02,2.163791000000e-01,0.000000000000e+00,0.000000000000e+00,1.000000000000e+00,2.745884000000e-03);
	Mat R0_rect = (Mat_<float>(4,4) << 9.999239000000e-01,9.837760000000e-03,-7.445048000000e-03,0,-9.869795000000e-03,9.999421000000e-01,-4.278459000000e-03,0,7.402527000000e-03,4.351614000000e-03,9.999631000000e-01,0, 0,0,0,1);
	Mat Tr_velo_to_cam = (Mat_<float>(4,4) << 7.533745000000e-03,-9.999714000000e-01,-6.166020000000e-04,-4.069766000000e-03,1.480249000000e-02,7.280733000000e-04,-9.998902000000e-01,-7.631618000000e-02,9.998621000000e-01,7.523790000000e-03,1.480755000000e-02,-2.717806000000e-01, 0,0,0,1);
	
	Mat trans = Mat(3,4,CV_32FC1);
	trans = P2 * R0_rect * Tr_velo_to_cam;
	Mat c_tmp = Mat(4,1,CV_32FC1);
	Mat p_result = Mat(1,3,CV_32FC1);
	
	cout << "trans = " << trans <<endl;

	for(int nIndex = 0; nIndex < point_cloud_ptr->points.size (); nIndex++)
	{	
		c_x = point_cloud_ptr->points[nIndex].x;
		c_y = point_cloud_ptr->points[nIndex].y;
		c_z = point_cloud_ptr->points[nIndex].z;
		
		c_tmp = (Mat_<float>(4, 1) << c_x, c_y, c_z, 1);		
		p_result = trans * c_tmp;
		
		p_w = p_result.at<float>(0,2);
		p_u = (int)((p_result.at<float>(0,0)) / p_w);
		p_v = (int)((p_result.at<float>(0,1)) / p_w);
		if( (p_u<0) || (p_u>cols)  || (p_v<0) || (p_v>rows) ||(p_w < 0)){
			point_cloud_ptr->points[nIndex].r = 128;
			point_cloud_ptr->points[nIndex].g = 2;
			point_cloud_ptr->points[nIndex].b = 64;
			continue;
		}
		point_cloud_ptr->points[nIndex].r = img.at<Vec3b>(p_v,p_u)[2];//not (p_u,p_v)!
		point_cloud_ptr->points[nIndex].g = img.at<Vec3b>(p_v,p_u)[1];
		point_cloud_ptr->points[nIndex].b = img.at<Vec3b>(p_v,p_u)[0];
	}	
	
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);    
    // 主循环  
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
