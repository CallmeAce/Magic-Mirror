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
namespace Map
{
	Class Magic_Mapping
	{
		private:
				const double resolution	= 0.1  // set the map resolution
				int Map_Size; // In the first version, the map is square,  and set the center as the origin
		public:

				/* Variables and elements */

				cv::Mat	Raw_Map;   // the global raw map, all the information of the objects will be present in this Mat formate image 
				typedef Eigen::Vector2f Glob_Point;// 2D point
				typedef pcl::PointXYZ PointT;
				typedef Eigen::Matrix2f G_R_Matrix; // 2D global rotation matrix
				struct  _Obj_Patch;// object(each cluster convex points) decomposition, store the information of each line of the object
				{
					Eigen::Vecotr2f Corner_p;
					int width;
					int height;
					double tan_value;
				};

				/*    GPS and gyroscope data structure */
				struct GPS_point // Gps and gyro scope information
				{
					double x;
					double z;
					double angle;
				}_gps;
						
				std::vector<Glob_Point> _g_points_v; // container of global object points
				std::vector<Obj_Patch> _patches_v; // container of patches(this is only for one cluster)
				
				/*  Class methods are defined here */		

				void Global_Transformation(std::vector<Glob_Point> & output, pcl::PointCloud<PointT>::Ptr & input_put);// transform the local coordinate to global coordination

};
	
	
}	





#endif
