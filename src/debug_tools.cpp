#include "cv_ext/types.h"
#include "cv_ext/debug_tools.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdexcept>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

int cv_ext::waitKeyboard( int delay )
{
  return cv::waitKey( delay )&0xFFFF;
}

cv::Vec3b cv_ext::randRGBColor()
{
  return cv::Vec3b(rand()%255, rand()%255, rand()%255);
}

template < typename _T > cv::Vec3b cv_ext::mapValue2RGB( _T val, _T max_val )
{
  // We take here inspiration from the HSV to RGB color conversion
  cv::Vec3b out;
  double h = double(val)/max_val;
  if( h >= 1.0 ) h = 0.99999999;
  h *= 4.0;
  uint32_t i = h;
  double f = h - i;
  double q = (1.0 - f);
  double t = f;

  switch(i)
  {
    case 0:
        out[0] = 255;
        out[1] = 255*t;
        out[2] = 0;
        break;
    case 1:
        out[0] = 255*q;
        out[1] = 255;
        out[2] = 0;
        break;
    case 2:
        out[0] = 0;
        out[1] = 255;
        out[2] = 255*t;
        break;
    case 3:
    default:
        out[0] = 0;
        out[1] = 255*q;
        out[2] = 255;
        break;
  }

  return out;
}

template < typename _T > void cv_ext::mapMat2RGB( const cv::Mat &src_mat, cv::Mat &dst_img, double max_val )
{
  if( dst_img.size() != src_mat.size() || dst_img.type() != cv::DataType<cv::Vec3b>::type )
    dst_img = cv::Mat(src_mat.size(), cv::DataType<cv::Vec3b>::type);
  for( int r = 0; r < src_mat.rows; r++ )
  {
    const _T *input_p = src_mat.ptr<_T>(r);
    cv::Vec3b *out_p = dst_img.ptr<cv::Vec3b>(r);
    for( int c = 0; c < src_mat.cols; c++, input_p++, out_p++ )
      *out_p = mapValue2RGB<_T>(*input_p, max_val );
  }
}

void cv_ext::showImage( const cv::Mat &img, const std::string &win_name, 
                        bool normalize, int sleep )
{
  if( img.channels() == 1 && normalize )
  {
    cv::Mat debug_img;
    cv::normalize(img, debug_img, 0, 255, cv::NORM_MINMAX, cv::DataType<uchar>::type);
    cv::imshow(win_name,debug_img);
  }
  else if( img.channels() == 3 && normalize )
  {
    cv::Mat debug_img;
    cv::normalize(img, debug_img, 0, 255, cv::NORM_MINMAX, cv::DataType<cv::Vec3b>::type);
    cv::imshow(win_name,debug_img);
  }
  else
    cv::imshow(win_name,img);

  if( sleep )
    cv_ext::waitKeyboard(sleep);
  else
    while ( cv_ext::waitKeyboard() != 27);
}


template <typename _TPoint3D> 
  void cv_ext::show3DPoints( const std::vector<_TPoint3D> &points, 
                             const std::string &win_name, bool show_axes, 
                             const cv_ext::vector_Isometry3d &axes_transf )
{
  int points_size = points.size();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );

  // Fill in the cloud data
  cloud->width    = points_size;
  cloud->height   = 1;
  cloud->is_dense = true;
  cloud->points.reserve ( points_size );

  for ( int i = 0; i < int(points_size); ++i )
    cloud->points.push_back( pcl::PointXYZ( points[i].x, points[i].y, points[i].z) );

  boost::shared_ptr<pcl::visualization::PCLVisualizer> 
    viewer (new pcl::visualization::PCLVisualizer (win_name));
    
  viewer->setBackgroundColor (0,0,0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud );
  if( show_axes )
  {
    if(axes_transf.empty())
      viewer->addCoordinateSystem (1.0);
    else
    {
      for(int i = 0; i < int(axes_transf.size()); i++)
        viewer->addCoordinateSystem (1.0, axes_transf[i].cast<float>());
    }
  }
  viewer->initCameraParameters ();
  
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
 boost::this_thread::sleep(boost::posix_time::milliseconds(100000)); }
  
  return;
}


#define CV_EXT_INSTANTIATE_show3DPoints(_T) \
template void cv_ext::show3DPoints( const std::vector< _T > &points, \
                                    const std::string &win_name, \
                                    bool show_axes, const cv_ext::vector_Isometry3d &axes_transf );
#define CV_EXT_INSTANTIATE_mapValue2RGB(_T) \
template cv::Vec3b cv_ext::mapValue2RGB( _T val, _T max_val );
#define CV_EXT_INSTANTIATE_mapMat2RGB(_T) \
template void cv_ext::mapMat2RGB<_T>( const cv::Mat &src_mat, cv::Mat &dst_img, double max_val );

CV_EXT_INSTANTIATE( show3DPoints, CV_EXT_3D_POINT_TYPES )
CV_EXT_INSTANTIATE( mapValue2RGB, CV_EXT_REAL_TYPES )
CV_EXT_INSTANTIATE( mapValue2RGB, CV_EXT_UINT_TYPES )
CV_EXT_INSTANTIATE( mapMat2RGB, CV_EXT_REAL_TYPES )
CV_EXT_INSTANTIATE( mapMat2RGB, CV_EXT_UINT_TYPES )
