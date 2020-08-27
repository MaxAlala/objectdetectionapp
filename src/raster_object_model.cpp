#include <iostream>
#include <iterator>
#include <boost/concept_check.hpp>
#include <algorithm>

#include "raster_object_model.h"

using namespace std;
using namespace cv;

RasterObjectModel::RasterObjectModel() :
  unit_meas_( MILLIMETER ),
  centroid_orig_offset_(USER_DEFINED_ORIG_OFFSET),
  orig_offset_(0.0f, 0.0f, 0.0f),
  step_(0.001),
  epsilon_(1e-4),
  min_seg_len_( numeric_limits< double >::min() )
{ 
    // rq_view == rotation quaternion
  rq_view_.setIdentity();
  t_view_.setZero();
}

void RasterObjectModel::setUnitOfMeasure( UoM val ) 
{ 
  unit_meas_ = val; 
}

void RasterObjectModel::setCentroidOrigOffset()
{
  centroid_orig_offset_ = CENTROID_ORIG_OFFSET;
}

void RasterObjectModel::setBBCenterOrigOffset()
{
  centroid_orig_offset_ = BOUNDING_BOX_CENTER_ORIG_OFFSET;
}

void RasterObjectModel::setOrigOffset( Point3f offset )
{ 
  orig_offset_ = offset;
  centroid_orig_offset_ = USER_DEFINED_ORIG_OFFSET;
}

void RasterObjectModel::setStepMeters( double s )
{ 
  step_ = s; 
}

void RasterObjectModel::setMinSegmentsLen ( double len )
{
  min_seg_len_ = len;
}

void RasterObjectModel::setCamModel ( const cv_ext::PinholeCameraModel& cam_model )
{
  cam_model_ = cam_model;
}

void RasterObjectModel::setModelView ( const double r_quat[4], const double t_vec[3] )
{
  rq_view_ = Eigen::Quaternion< double, Eigen::DontAlign >( r_quat[0], r_quat[1], r_quat[2], r_quat[3] );
  t_view_ = Eigen::Map<const Eigen::Vector3d>(t_vec);

  updateRaster();
}

void RasterObjectModel::setModelView ( const Eigen::Quaterniond& r_quat, const Eigen::Vector3d& t_vec )
{
  rq_view_ = r_quat;
  t_view_ = t_vec;
  
  updateRaster();
}

void RasterObjectModel::setModelView ( const Mat_< double >& r_vec, const Mat_< double >& t_vec )
{
  double r_quat[4];
  ceres::AngleAxisToQuaternion( (const double*)(r_vec.data), r_quat) ;
  rq_view_ = Eigen::Quaterniond( r_quat[0], r_quat[1], r_quat[2], r_quat[3] );
  t_view_ = Eigen::Map< const Eigen::Vector3d>( (const double*)(t_vec.data) );
  
  updateRaster();
}

void RasterObjectModel::setModelView ( int idx )
{
    // assign rotation and translation vector
  rq_view_ = precomputed_rq_view_[idx];
  t_view_ = precomputed_t_view_[idx];
  retreiveModel( idx );
}

void RasterObjectModel::storeModelView()
{
  precomputed_rq_view_.push_back(rq_view_);
  precomputed_t_view_.push_back(t_view_);
  storeModel();
}
void RasterObjectModel::removeModelView(int id)
{
  precomputed_rq_view_.erase(precomputed_rq_view_.begin() + id);
  precomputed_t_view_.erase(precomputed_t_view_.begin() + id);
  
}

void RasterObjectModel::removeModelView()
{
  precomputed_rq_view_.pop_back();
  precomputed_t_view_.pop_back();
  
}
void RasterObjectModel::modelView ( double r_quat[4], double t_vec[3] ) const
{
  r_quat[0] = rq_view_.w();
  r_quat[1] = rq_view_.x();
  r_quat[2] = rq_view_.y();
  r_quat[3] = rq_view_.z();

  t_vec[0] = t_view_(0);
  t_vec[1] = t_view_(1);
  t_vec[2] = t_view_(2);
}

void RasterObjectModel::modelView ( Eigen::Quaterniond& r_quat, Eigen::Vector3d& t_vec ) const
{
  r_quat = rq_view_;
  t_vec = t_view_;
}

void RasterObjectModel::modelView ( int idx, Eigen::Quaterniond& r_quat, Eigen::Vector3d& t_vec ) const
{
  r_quat = precomputed_rq_view_[idx];
  t_vec = precomputed_t_view_[idx];
}

void RasterObjectModel::modelView ( Mat_< double >& r_vec, Mat_< double >& t_vec ) const
{
  r_vec = Mat_< double >(3,1);
  t_vec = Mat_< double >(3,1);
  
  double r_quat[4];
  r_quat[0] = rq_view_.w();
  r_quat[1] = rq_view_.x();
  r_quat[2] = rq_view_.y();
  r_quat[3] = rq_view_.z();

  ceres::QuaternionToAngleAxis<double> ( r_quat, ( double* )r_vec.data );
  
  t_vec(0) = t_view_(0);
  t_vec(1) = t_view_(1);
  t_vec(2) = t_view_(2);
}


void RasterObjectModel::projectRasterPoints( vector<Point2f> &proj_pts,
                                             bool only_visible_points ) const
{
  projectRasterPoints( rq_view_, t_view_, proj_pts, only_visible_points );
}
                                             
void RasterObjectModel::projectRasterPoints( const double r_quat[4], const double t_vec[3], 
                                             vector<Point2f> &proj_pts,
                                             bool only_visible_points )  const
{
  const vector<Point3f> &pts = getPoints(only_visible_points);

  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
}

void RasterObjectModel::projectRasterPoints( const Eigen::Quaterniond &r_quat, const Eigen::Vector3d &t_vec, 
                                             vector<Point2f> &proj_pts, bool only_visible_points )  const
{
  const vector<Point3f> &pts = getPoints(only_visible_points);
    
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
}

void RasterObjectModel::projectRasterPoints( const Mat_<double> &r_vec, const Mat_<double> &t_vec,
                                             vector<Point2f> &proj_pts, bool only_visible_points )  const
{
  const vector<Point3f> &pts = getPoints(only_visible_points);
    
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_vec, t_vec );
  proj.projectPoints( pts, proj_pts );
}

void RasterObjectModel::projectRasterPoints ( int idx, vector< Point2f >& proj_pts ) const
{
  projectRasterPoints ( idx, precomputed_rq_view_[idx], precomputed_t_view_[idx], proj_pts );
}

void RasterObjectModel::projectRasterPoints ( int idx, const double r_quat[4], const double t_vec[3],
                                              vector< Point2f >& proj_pts ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
}

void RasterObjectModel::projectRasterPoints ( int idx, const Eigen::Quaterniond &r_quat, const Eigen::Vector3d &t_vec,
                                              vector< Point2f >& proj_pts ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
}

void RasterObjectModel::projectRasterPoints ( int idx, const Mat_<double> &r_vec, const Mat_<double> &t_vec,
                                              vector< Point2f >& proj_pts ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( r_vec, t_vec );
  proj.projectPoints( pts, proj_pts );
}
 
void RasterObjectModel::projectRasterPoints( vector<Point2f> &proj_pts,
                                             vector<float> &normal_directions,
                                             bool only_visible_points )  const
{  
  projectRasterPoints( rq_view_, t_view_, proj_pts, normal_directions, only_visible_points );
}


void RasterObjectModel::projectRasterPoints( const double r_quat[4], const double t_vec[3],
                                             vector<Point2f> &proj_pts,
                                             vector<float> &normal_directions,
                                             bool only_visible_points )  const
{
  const vector<Point3f> &pts = getPoints(only_visible_points);
  const vector<Point3f> &d_pts = getDPoints(only_visible_points);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );

  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}

void RasterObjectModel::projectRasterPoints( const Eigen::Quaterniond &r_quat, const Eigen::Vector3d &t_vec,
                                             vector<Point2f> &proj_pts,
                                             vector<float> &normal_directions,
                                             bool only_visible_points )  const
{  
  const vector<Point3f> &pts = getPoints(only_visible_points);
  const vector<Point3f> &d_pts = getDPoints(only_visible_points);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );

  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}

void RasterObjectModel::projectRasterPoints( const Mat_<double> &r_vec, const Mat_<double> &t_vec,
                                             vector<Point2f> &proj_pts,
                                             vector<float> &normal_directions,
                                             bool only_visible_points )  const
{  
  const vector<Point3f> &pts = getPoints(only_visible_points);
  const vector<Point3f> &d_pts = getDPoints(only_visible_points);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );

  proj.setTransformation( r_vec, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}


void RasterObjectModel::projectRasterPoints ( int idx, vector< Point2f >& proj_pts,
                                              vector< float >& normal_directions ) const
{
  projectRasterPoints( idx, precomputed_rq_view_[idx], precomputed_t_view_[idx], 
                       proj_pts, normal_directions );
}

void RasterObjectModel::projectRasterPoints ( int idx, const double r_quat[4], const double t_vec[3],
                                              vector< Point2f >& proj_pts,
                                              vector< float >& normal_directions ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  const vector<Point3f> &d_pts = getPrecomputedDPoints(idx);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );

  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}

void RasterObjectModel::projectRasterPoints ( int idx, const Eigen::Quaterniond &r_quat, const Eigen::Vector3d &t_vec,
                                              vector< Point2f >& proj_pts,
                                              vector< float >& normal_directions ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  const vector<Point3f> &d_pts = getPrecomputedDPoints(idx);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );

  
  proj.setTransformation( r_quat, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}

void RasterObjectModel::projectRasterPoints ( int idx, const Mat_<double> &r_vec, const Mat_<double> &t_vec,
                                              vector< Point2f >& proj_pts,
                                              vector< float >& normal_directions ) const
{
  const vector<Point3f> &pts = getPrecomputedPoints(idx);
  const vector<Point3f> &d_pts = getPrecomputedDPoints(idx);
  vector<Point2f> d_proj_pts;
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  
  proj.setTransformation( r_vec, t_vec );
  proj.projectPoints( pts, proj_pts );
  proj.projectPoints( d_pts, d_proj_pts );
  
  int pts_size = pts.size();
  normal_directions.resize(pts_size);
  
  for( int i = 0; i < pts_size; i++)
  {
    Point2f diff(d_proj_pts[i] - proj_pts[i]);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }
}


void RasterObjectModel::projectRasterSegments ( vector< Vec4f >& proj_segs,
                                                bool only_visible_segments  ) const
{
  const vector<Vec6f> &segs = getSegments(only_visible_segments);
  
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( rq_view_, t_view_ );
  proj.projectSegments( segs, proj_segs );
}

void RasterObjectModel::projectRasterSegments ( vector< Vec4f >& proj_segs,
                                                vector< float >& normal_directions,
                                                bool only_visible_segments ) const
{
  const vector<Vec6f> &segs = getSegments(only_visible_segments);
  const vector<Point3f> &d_segs = getDSegments(only_visible_segments);
  vector<Point2f> d_proj_segs;
    
  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( rq_view_, t_view_ );
  
  proj.projectSegments( segs, proj_segs );
  proj.projectPoints( d_segs, d_proj_segs );

  int seg_size = segs.size();
  normal_directions.resize(seg_size);
  
  for( int i = 0; i < seg_size; i++)
  {
    Vec4f &raster_seg = proj_segs[i];
    Point2f p0( raster_seg[0], raster_seg[1] );
    Point2f diff(d_proj_segs[i] - p0);
    if( diff.y )
      normal_directions[i] = -atan( diff.x/diff.y );
    else
      normal_directions[i] = -M_PI/2;
  }  
}

Mat RasterObjectModel::getMask ()
{

  Mat mask, tmp_mask[2];

  tmp_mask[0] = Mat::zeros(cam_model_.imgSize(), DataType< uchar >::type );
  vector<Vec4f> proj_segs;
  projectRasterSegments( proj_segs,false );
  cv_ext::drawSegments( tmp_mask[0], proj_segs );
  transpose(tmp_mask[0], tmp_mask[1]);

  int rows = cam_model_.imgHeight(), cols = cam_model_.imgWidth();
  for( int i = 0; i < 2; i++ )
  {
    if(i)
    {
      rows = cam_model_.imgWidth();
      cols = cam_model_.imgHeight();
    }

    for( int r = 0; r < rows; r++ )
    {
      uchar *mask_p = tmp_mask[i].ptr<uchar>(r);
      int start_c = -1, end_c = -1;
      for( int c = 0; c < cols; c++, mask_p++ )
      {
        if( *mask_p )
        {
          start_c = c;
          break;
        }
      }
      mask_p = tmp_mask[i].ptr<uchar>(r);
      mask_p += cols - 1;
      for( int c = cols - 1; c >= 0 ; c--, mask_p-- )
      {
        if( *mask_p )
        {
          end_c = c;
          break;
        }
      }
      if( start_c >= 0 && end_c >= 0 )
      {
        mask_p = tmp_mask[i].ptr<uchar>(r);
        mask_p += start_c;
        for( int c = start_c; c <= end_c; c++, mask_p++ )
          *mask_p = 255;
      }
    }
  }
  transpose(tmp_mask[1], mask);
  mask &= tmp_mask[0];

  return mask;
}

void RasterObjectModel::loadPrecomputedModelsViews ( string filename )
{
  precomputed_rq_view_.clear();
  precomputed_t_view_.clear();
 
  int pos1 = filename.length() - 4, pos2 = filename.length() - 5;
  std::string ext_str1 = filename.substr ((pos1 > 0)?pos1:0);
  std::string ext_str2 = filename.substr ((pos2 > 0)?pos2:0);

  if( ext_str1.compare(".yml") && ext_str2.compare(".yaml") &&
      ext_str1.compare(".YML") && ext_str2.compare(".YAML") )
    filename += ".yml";
  
  FileStorage fs(filename.data(), FileStorage::READ);
  
  Mat model_views;
  fs["model_views"] >> model_views;
  
  for( int i = 0; i< model_views.rows; i++ )
  {
    Eigen::Quaterniond quat(model_views.at<double>(i,0),
                            model_views.at<double>(i,1),
                            model_views.at<double>(i,2),
                            model_views.at<double>(i,3));
    Eigen::Vector3d t(model_views.at<double>(i,4),
                      model_views.at<double>(i,5),
                      model_views.at<double>(i,6));
    
    precomputed_rq_view_.push_back(quat);
    precomputed_t_view_.push_back(t);
  }
  
  loadPrecomputedModels(fs);
  
  fs.release();
}

void RasterObjectModel::savePrecomputedModelsViews ( string filename ) const
{
  Mat model_views(precomputed_rq_view_.size(), 7, DataType<double>::type );
  for( int i = 0; i < int(precomputed_rq_view_.size()); i++ )
  {
    model_views.at<double>(i,0) = precomputed_rq_view_[i].w();
    model_views.at<double>(i,1) = precomputed_rq_view_[i].x();
    model_views.at<double>(i,2) = precomputed_rq_view_[i].y();
    model_views.at<double>(i,3) = precomputed_rq_view_[i].z();
    model_views.at<double>(i,4) = precomputed_t_view_[i](0);
    model_views.at<double>(i,5) = precomputed_t_view_[i](1);
    model_views.at<double>(i,6) = precomputed_t_view_[i](2);
  }
  
  int pos1 = filename.length() - 4, pos2 = filename.length() - 5;
  std::string ext_str1 = filename.substr ((pos1 > 0)?pos1:0);
  std::string ext_str2 = filename.substr ((pos2 > 0)?pos2:0);

  if( ext_str1.compare(".yml") && ext_str2.compare(".yaml") &&
      ext_str1.compare(".YML") && ext_str2.compare(".YAML") )
    filename += ".yml";
  
  FileStorage fs(filename.data(), FileStorage::WRITE);
  
  fs << "model_views" << model_views;
  
  savePrecomputedModels(fs);
  
  fs.release();
}


void RasterObjectModel::projectAxes ( vector< Vec4f >& proj_segs_x, 
                                      vector< Vec4f >& proj_segs_y, 
                                      vector< Vec4f >& proj_segs_z, 
                                      double lx, double ly, double lz, double radius)
{
  double arrow_len = radius*10;
  const int ang_step = 18;

  cv_ext::PinholeSceneProjector proj( cam_model_ );
  proj.setTransformation( rq_view_, t_view_ );
  
  if(radius <= 0) 
    radius = 0.001;
  
  if(lx > 0)
  {
    vector<Vec6f> segs_x;
    vector<Point3f> circle_pts_x, arrow_circle_pts_x;
    
    circle_pts_x.reserve(ang_step);

    arrow_circle_pts_x.reserve(ang_step);
    
    segs_x.reserve(ang_step*5);
        
    double ang = 0, d_ang = 2*M_PI/ang_step;
    for(int i = 0; i < ang_step; i++, ang += d_ang )
    {
      circle_pts_x.push_back(Point3f(0, radius*cos(ang), radius*sin(ang)));      
      arrow_circle_pts_x.push_back(Point3f(0, 3*radius*cos(ang), 3*radius*sin(ang)));
    }
    
    for(int i0 = 0; i0 < ang_step; i0++  )
    {
      int i1 = (i0+1)%ang_step;
      segs_x.push_back( Vec6f(circle_pts_x[i0].x, circle_pts_x[i0].y, circle_pts_x[i0].z,
                              circle_pts_x[i1].x, circle_pts_x[i1].y, circle_pts_x[i1].z) );

      segs_x.push_back( Vec6f(lx + circle_pts_x[i0].x, circle_pts_x[i0].y, circle_pts_x[i0].z,
                              lx + circle_pts_x[i1].x, circle_pts_x[i1].y, circle_pts_x[i1].z) );
      
      segs_x.push_back( Vec6f(circle_pts_x[i0].x, circle_pts_x[i0].y, circle_pts_x[i0].z,
                              lx + circle_pts_x[i0].x, circle_pts_x[i0].y, circle_pts_x[i0].z) );
      
      segs_x.push_back( Vec6f(lx + arrow_circle_pts_x[i0].x, arrow_circle_pts_x[i0].y, arrow_circle_pts_x[i0].z,
                              lx + arrow_circle_pts_x[i1].x, arrow_circle_pts_x[i1].y, arrow_circle_pts_x[i1].z) );

      segs_x.push_back( Vec6f(lx + arrow_circle_pts_x[i0].x, arrow_circle_pts_x[i0].y, arrow_circle_pts_x[i0].z,
                              lx + arrow_len, 0,0 ) );
    }
    
    proj.projectSegments( segs_x, proj_segs_x );
  }
  
  if(ly > 0)
  {
    vector<Vec6f> segs_y;
    vector<Point3f> circle_pts_y, arrow_circle_pts_y;
    
    circle_pts_y.reserve(ang_step);
    arrow_circle_pts_y.reserve(ang_step);
    
    segs_y.reserve(ang_step*5);
        
    double ang = 0, d_ang = 2*M_PI/ang_step;
    for(int i = 0; i < ang_step; i++, ang += d_ang )
    {
      circle_pts_y.push_back(Point3f(radius*cos(ang), 0, radius*sin(ang)));
      arrow_circle_pts_y.push_back(Point3f(3*radius*cos(ang), 0, 3*radius*sin(ang)));
    }
    
    for(int i0 = 0; i0 < ang_step; i0++  )
    {
      int i1 = (i0+1)%ang_step;

      segs_y.push_back( Vec6f(circle_pts_y[i0].x, circle_pts_y[i0].y, circle_pts_y[i0].z,
                              circle_pts_y[i1].x, circle_pts_y[i1].y, circle_pts_y[i1].z) );

      segs_y.push_back( Vec6f(circle_pts_y[i0].x, ly + circle_pts_y[i0].y, circle_pts_y[i0].z,
                              circle_pts_y[i1].x, ly + circle_pts_y[i1].y, circle_pts_y[i1].z) );

      segs_y.push_back( Vec6f(circle_pts_y[i0].x, circle_pts_y[i0].y, circle_pts_y[i0].z,
                              circle_pts_y[i0].x, ly + circle_pts_y[i0].y, circle_pts_y[i0].z) );
      
      segs_y.push_back( Vec6f(arrow_circle_pts_y[i0].x, ly + arrow_circle_pts_y[i0].y, arrow_circle_pts_y[i0].z,
                              arrow_circle_pts_y[i1].x, ly + arrow_circle_pts_y[i1].y, arrow_circle_pts_y[i1].z) );

      segs_y.push_back( Vec6f(arrow_circle_pts_y[i0].x, ly + arrow_circle_pts_y[i0].y, arrow_circle_pts_y[i0].z,
                              0, ly + arrow_len, 0) );
    }
    
    proj.projectSegments( segs_y, proj_segs_y );
  }
  
  if(lz > 0)
  {
    vector<Vec6f> segs_z;
    vector<Point3f> circle_pts_z, arrow_circle_pts_z;
    

    circle_pts_z.reserve(ang_step);
    arrow_circle_pts_z.reserve(ang_step);
    
    segs_z.reserve(ang_step*5);
        
    double ang = 0, d_ang = 2*M_PI/ang_step;
    for(int i = 0; i < ang_step; i++, ang += d_ang )
    {
      circle_pts_z.push_back(Point3f(radius*cos(ang), radius*sin(ang), 0));
      arrow_circle_pts_z.push_back(Point3f(3*radius*cos(ang), 3*radius*sin(ang), 0));    
    }
    
    for(int i0 = 0; i0 < ang_step; i0++  )
    {
      int i1 = (i0+1)%ang_step;
      segs_z.push_back( Vec6f(circle_pts_z[i0].x, circle_pts_z[i0].y, circle_pts_z[i0].z,
                              circle_pts_z[i1].x, circle_pts_z[i1].y, circle_pts_z[i1].z) );

      segs_z.push_back( Vec6f(circle_pts_z[i0].x, circle_pts_z[i0].y, lz + circle_pts_z[i0].z,
                              circle_pts_z[i1].x, circle_pts_z[i1].y, lz + circle_pts_z[i1].z) );
      
      segs_z.push_back( Vec6f(circle_pts_z[i0].x, circle_pts_z[i0].y, circle_pts_z[i0].z,
                              circle_pts_z[i0].x, circle_pts_z[i0].y, lz + circle_pts_z[i0].z) );
      
      segs_z.push_back( Vec6f(arrow_circle_pts_z[i0].x, arrow_circle_pts_z[i0].y, lz + arrow_circle_pts_z[i0].z,
                              arrow_circle_pts_z[i1].x, arrow_circle_pts_z[i1].y, lz + arrow_circle_pts_z[i1].z) );

      segs_z.push_back( Vec6f(arrow_circle_pts_z[i0].x, arrow_circle_pts_z[i0].y, lz + arrow_circle_pts_z[i0].z,
                              0, 0, lz + arrow_len) );    
    }
    
    proj.projectSegments( segs_z, proj_segs_z );
  } 
}
