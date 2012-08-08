#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZ PointT;
void extract_plane(pcl::PointCloud<PointT>::Ptr & inputcloud,pcl::PointCloud<PointT>::Ptr & outputcloud,Eigen::VectorXf & plane_coefficients,double threshold)
{
  if (plane_coefficients.size() != 4)
   {
     printf ("[pcl::SampleConsensusModelPlane::countWithinDistance] Invalid number of model coefficients given (%zu)!\n", plane_coefficients.size ());
   }
 
 
   // Iterate through the 3d points and calculate the distances from them to the plane
   for (size_t i = 0; i < inputcloud->points.size(); ++i)
   {
     // Calculate the distance from the point to the plane normal as the dot product
     // D = (P-A).N/|N|
     Eigen::Vector4f pt (inputcloud->points[i].x,
                         inputcloud->points[i].y,
                         inputcloud->points[i].z,
                         1);
     if (std::fabs (plane_coefficients.dot (pt)) < threshold)
       outputcloud->points.push_back(inputcloud->points[i]); 

  }
 };

// point cloud rotation acording to plane coefficient
// the groud plane is not (0,1,0,0) depends on how and where you mounte camera.
void points_rotation(pcl::PointCloud<PointT>::Ptr & input_cloud, pcl::PointCloud<PointT>::Ptr & output_cloud, Eigen::VectorXf & coefficient)

{
//  build the Euler angle roation matrix from the plane coefficent aX + bY + cZ + D = 0

//  Rz(a) = [cos(a) -sin(a) 0;   Ry(a) = [cos(a)  0 sin(a);   Rx(a) = [1     0          0;    
//           sin(a)  cos(a) 0;             0      1    0  ;            0  cos(a)  -sin(a);
//           0        0     1]            -sin(a) 0 cos(a)]            0  sin(a)   cos(a)]


// General rotation matrix 
// Rz(a1)R(a2)R(a3)  = [cos(a2)cos(a1)  -cos(a3)sin(a1)+sin(a3)sin(a2)cos(a1)  sin(a3)sin(a1)+cos(a3)sin(a2)cos(a1);
//                      cos(a2)sin(a1)   cos(a3)cos(a1)+sin(a3)sin(a2)sin(a1) -sin(a3)cos(a1)+cos(a3)sin(a2)sin(a1);
//                        -sin(a2)                  sin(a3)sin(a2)                           cos(a3)cos(a2)        ]

// compute angle 
  float a_z = -atan2( coefficient[0],coefficient[1]); // rotation around z axis   
  float a_y;
  if(a_z== 0)
  a_y = -atan2(coefficient[0],coefficient[2]);
  else
  a_y = -atan2( coefficient[2],coefficient[0]); // rotation around y axis   
  
  float a_x = -atan2( coefficient[2],coefficient[1]); // rotation around z axis   
  float wat = atan2(0,0);
  std::cout<< "angles are: "<< a_z <<","<< a_y <<"," <<a_x <<","<<wat<< std::endl; 
//   Eigen::MatrixXf tran_Matrix_z(3,3); // transition matrix_z
//   
//   Eigen::MatrixXf tran_Matrix_y(3,3); // transition matrix_y
//   
//   Eigen::MatrixXf tran_Matrix_x(3,3); // transition matrix_x
   
   Eigen::Matrix3f rot_Matrix_z; // transition matrix_z
   
   Eigen::Matrix3f rot_Matrix_y; // transition matrix_y
   
   Eigen::Matrix3f rot_Matrix_x; // transition matrix_x
   
   Eigen::Matrix3f Rot_Matrix; // transition matrix_x
 
   rot_Matrix_z <<   cos(a_z), -sin(a_z),  0,
                     sin(a_z),  cos(a_z),  0,
                     0,         0,         1;
//
//   rot_Matrix_y <<   cos(a_y),  0,    sin(a_y),
//                     0,         1,           0,
//                    -sin(a_y),  0,    cos(a_y);
       
   rot_Matrix_x <<   1,         0,           0,
                     0,  cos(a_x),   -sin(a_x),
                     0,  sin(a_x),    cos(a_x);
   
//   Rot_Matrix  = rot_Matrix_z * rot_Matrix_y * rot_Matrix_x;

   Rot_Matrix  = rot_Matrix_z * rot_Matrix_x;
// move all the points along the direction of the normal of the plane with "d_t_0", where d is the distance from original point (0,0,0) to the plane
   
   float d_t_0 = coefficient[3];
  
   Eigen::Vector3f point;
   Eigen::Vector3f trans_d;
   
   //normalize the plane norm
//-------------------------------------------------------
   Eigen::Vector3f trans_dd;
   Eigen::Vector4f coefficient_norm;
   coefficient_norm = coefficient;
   float sq_sum_1 = coefficient_norm.norm();
   std::cout<< "norm is "<< sq_sum_1<<std::endl;
//--------------------------------------------------------
   Eigen::Vector3f coefficient_norm_1;
   //float sq_sum  = sqrt(coefficient[0]*coefficient[0]+coefficient[1]*coefficient[1]+coefficient[2]*coefficient[2]+coefficient[3]*coefficient[3]);
  
   float sq_sum  = sqrt(coefficient[0]*coefficient[0]+coefficient[1]*coefficient[1]+coefficient[2]*coefficient[2]);
   coefficient_norm_1[0] = coefficient[0]/sq_sum;
   coefficient_norm_1[1] = coefficient[1]/sq_sum;
   coefficient_norm_1[2] = coefficient[2]/sq_sum;
   d_t_0                 = coefficient[3];
   trans_d             = coefficient_norm_1 * d_t_0;
 
   // point container
   pcl::PointXYZ temp_point;

   for(unsigned int i = 0; i< input_cloud->points.size();i ++)
   {
     point << input_cloud->points[i].x,
              input_cloud->points[i].y,
              input_cloud->points[i].z;
   
     point = Rot_Matrix * point;
     
     point = point + trans_d;
     
     temp_point.x = point[0];
     temp_point.y = point[1];
     temp_point.z = point[2];

     output_cloud->points.push_back(temp_point);

   }


}










const std::vector<int>  indices;
//---------------------small test---------------------------
class A
{
 public:
 int s;
 int w;
 A():s(100),b(100),w(100){};//constract
 void foo(){ 
 s = 1;
 b = 2;
 printf("%d,%d,%d",s,w,b);

 };
private:
 int b;
}a;
//---------------------------------------------------------



int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>),
                                      cloud_rotate (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointXYZ flv_point;// first point define the line
  pcl::PointXYZ slv_point;// second point define the line
  pcl::PointXYZ flh_point;// first point define the line
  pcl::PointXYZ slh_point;// second point define the line
  
  Eigen::VectorXf model_coefficient(4);
  Eigen::VectorXf norm_model_coefficient(4);
  std::vector<double> distance;
  pcl::visualization::PCLVisualizer viewer;
//---------------------------RACsegment-----------------------------------------
//pcl::SACSegmentation<pcl::PointXYZ> seg;
//  seg.setOptimizeCoefficients (false);
//  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
//  seg.setMethodType (pcl::SAC_RANSAC);
//  seg.setMaxIterations (100);
//  seg.setDistanceThreshold (0.01);
//  seg.setAxis(Eigen::Vector3f(0,0,1));
//  seg.setEpsAngle(0.01);
//-------------------------------------------------------------------------
  pcl::SampleConsensusModelPlane<PointT> plane_extractor(cloud,indices);
  reader.read ("plane_sample.pcd", *cloud); 
std::cout<<cloud->points.size()<<std::endl;
 
 float a_1,b_1,c_1,d_1;
  std::cout<< "please input the plane parameters"<< std::endl;
  std::cin >> a_1 >> b_1 >> c_1 >> d_1;

  model_coefficient[0] = a_1;
  
  model_coefficient[1] = b_1;
  
  model_coefficient[2] = c_1;
  
  model_coefficient[3] = d_1;
  
//plane roation
  points_rotation(cloud,cloud_rotate,model_coefficient);
  std::cout<< "size of r cloud"<< cloud_rotate->points.size()<<std::endl;
 // writer.write("rotate_points.pcd", *cloud_rotate,false);
  for (int i = 0; i < model_coefficient.size(); i++)
  {
    std::cout<<model_coefficient[i]<<std::endl;
 
  }
  float f = model_coefficient.norm();
  model_coefficient.normalize();
  std::cout <<"waht"<< model_coefficient[1]<< f<<std::endl;
  for (int i=0;i<4;i++)
  {
    std::cout<<model_coefficient[i]<<std::endl;
 
  }

  extract_plane(cloud,cloud_filtered,model_coefficient,3);

  
  std::cout<<"number of plane points: "<<cloud_filtered->points.size()<<std::endl; 
 
//  std::cout<<"......fuck eigen"<<model_coefficient.size()<<std::endl;  
//  std::cout<<plane_extractor.countWithinDistance(model_coefficient,10)<<std::endl;
//  
//  //std::cout<<" the number of the plane inliners : "<< inlier <<std::endl;
//  plane_extractor.getDistancesToModel(model_coefficient,distance);
//  for(int i = 0; i< distance.size();i++)
//  {
//    std::cout<<" list all the distances : "<< distance[i]<<std::endl;
//  }
  
//a.foo();

std::cout<<a.s<<std::endl;
//std::cout<<a.b<<std::endl;
  viewer.addPointCloud<pcl::PointXYZ>(cloud_rotate,"1");
  viewer.addPointCloud<pcl::PointXYZ>(cloud,"2");
  viewer.addCoordinateSystem(1);
// define the line
  char str[100];
  for(unsigned int i = 0; i < 200; i++)
  {
    flh_point.x = -40;// draw the horizontal grid coordinate
    flh_point.y = 0;
    flh_point.z = i*0.4;
    slh_point.x = 40;
    slh_point.y = 0;
    slh_point.z = i*0.4;

    flv_point.x = i*0.4-40;// draw the vertical grid coordinate
    flv_point.y = 0;
    flv_point.z = 0;
    slv_point.x = i*0.4-40;
    slv_point.y = 0;
    slv_point.z = 80;
    sprintf(str,"grid vertical%03d",i);
//  draw the grid line on the visualizer
    viewer.addLine<pcl::PointXYZ> (flh_point,slh_point,0,0,1,str);
 
    sprintf(str,"grid horizontal%03d",i);
    viewer.addLine<pcl::PointXYZ> (flv_point,slv_point,0,0,1,str);
  }


while(!viewer.wasStopped())
{
  viewer.spinOnce(100);
}

}

