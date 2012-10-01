#ifndef MAGIC_MAPPING_H
#define MAGIC_MAPPING_H
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <Eigen/Dense>
/*    Magic Mapping   */
//namespace Map
//{
#define pi 3.14	
class Magic_Mapping
{
		private:
				const static double resolution	= 0.01;  // set the map resolution
				int Map_Size; // In the first version, the map is square,  and set the center as the origin
		public:

				/* Variables and elements */
				int counter;
                int thresh_counter;
				cv::Mat	Raw_Map;   // the global raw map, all the information of the objects will be present in this Mat formate image 
				typedef Eigen::Vector2f Glob_Point;// 2D point
				typedef pcl::PointXYZ PointT;
				typedef Eigen::Matrix2f G_R_Matrix; // 2D global rotation matrix
				struct  Obj_Patch// object(each cluster convex points) decomposition, store the information of each line of the object
				{
					Eigen::Vector2f Corner_p;
					int width;
					int height;
					double tan_value;
				};

				struct Obj_Patch_1

				{
					Eigen::Vector2f Corner_p;
					int width;
					int height;
		
		//			double tan_value;
				};
				struct P_index
				{
					int x;
					int y;
				};
				/*    GPS and gyroscope data structure */
				struct GPS_point // Gps and gyro scope information
				{
					double x ;
					double z ;
					double angle;
				}_gps;
						
				std::vector<Glob_Point> _g_points_v; // container of global object points
				std::vector<Obj_Patch,Eigen::aligned_allocator<Obj_Patch> > _patches_v; // container of patches(this is only for one cluster)
				std::vector<Eigen::Vector2f> _g_ind_v;   // bit map indices of the global points	
				/* constructor */
				
				Magic_Mapping ();
				~Magic_Mapping ();

				/*  Class methods are defined here */		
				void Map_Init();// initialize the Map;
				void Global_Transformation (std::vector<Glob_Point> & output, pcl::PointCloud<PointT>::Ptr & input_put);// transform the local coordinate to global coordination
				
				void Get_Patch (Glob_Point & p_1, Glob_Point & p_2, Obj_Patch & output);// get the bounding box patch from two points
				// function overload
				void Get_Patch (std::vector<Glob_Point> & input_points, std::vector<Obj_Patch,Eigen::aligned_allocator<Obj_Patch> > & output_points);

//TODO start
/*     Get Patch of the whole cluster, and do the fill-in operations   */
				int To_right (Eigen::Vector2f & p1, Eigen::Vector2f & p2, Eigen::Vector2f & p3);  
				void P_Index(std::vector<Glob_Point>& input_points, std::vector<Eigen::Vector2f> & output); // get the indices of points in the map

				void Get_Patch (std::vector<Glob_Point> & input_points, Obj_Patch_1 & output);
// Inject values into covex realm
				void Patch_Injection (Obj_Patch_1 & cluster_patch, std::vector<Eigen::Vector2f> & input_patches, float & weight, cv::Mat & output);
				
				void Bresenham_line(int  x0, int  y0, int  x1, int  y1, float & weight, cv::Mat & output);				

				void Bresenham (Obj_Patch & input, float & weight, cv::Mat & output);

//TODO ending

				void Tri_Grid (Obj_Patch & input, float & weight, cv::Mat & output); // Build the triangle occupancy map

				void Map_Update (cv::Mat & input_Map, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > & input_points, float & weight );
 // add the occupancy patch to the global map

				void Map_Thresholding (cv::Mat & Map, float  & threshold); // remove the noise from the global map

};
	
	
//}	





#endif
