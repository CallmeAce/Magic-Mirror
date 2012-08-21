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
// Data point
typedef Eigen::Vector2f Glob_Point;
typedef pcl::PointXYZ PointT;
typedef Eigen::Matrix2f G_R_Matrix;
const double resolution = 0.1;    // map resolution
int Map_Size = 600;

struct Obj_Patch
{
	Eigen::Vector2f Corner_p;
	int width;
	int height;
    double tan_value;
};

// GPS data structure
struct GPS_point
{
	double x;
	double z;
	double angle;
}gps;



std::vector<Glob_Point> glob_obj_points;  // global object points

std::vector<Obj_Patch> obj_patches;// store obj patches (attention: only one obj)


// The overall map
cv::Mat raw_map;




// Global transfermation 
void Global_tran (std::vector<Glob_Point> & output, pcl::PointCloud<PointT>::Ptr & input_point)
{
	// rotation matrix around y axis
	G_R_Matrix r_matrix;
	r_matrix <<  cos(-gps.angle), sin(-gps.angle),
			  	-sin(-gps.angle), cos(-gps.angle);
    // transition vector
	Glob_Point trans_point;
	trans_point << gps.x, gps.z;
    // put x, z data into the Glob point structure 
	Glob_Point temp_point;
	// push_back the points
	for(unsigned int i = 0; i< input_point->points.size(); ++i)
	{	
		temp_point << input_point->points[i].x,
	 	input_point->points[i].z;
		
//		temp_point = temp_point + trans_point;	
		temp_point = r_matrix * temp_point;
		temp_point = temp_point + trans_point;	
		output.push_back(temp_point);
		std::cout<<"global points ff"<<temp_point<<std::endl;
	}
}



// bounding box
//std::vector<PointT> extream_points (pcl::PointCloud<PointT>::Ptr & input_point)
//{
//    PointT top    = input_point[0];
//    PointT bottom = input_point[0];
//    PointT left   = input_point[0];
//    PointT right  = input_point[0];
//
//
//	int size = input_point->points.size();
//	for (unsigned int i = 0; i< size; i++)
//	{
//		if(top.z < input_point->points[i])
//		   top     = input_point->points[i];
//		if(bottom.z > input_point->points[i])
//		   bottom  = input_point->points[i];
//		if(left.x > input_point->points[i])
//		   left    = input_point->points[i];
//		if(right.x < input_point->points[i])
//		   right   = input_point->points[i];
//	}
//}


// get the bounding box patch from two points
void Get_patch (Glob_Point & p_1, Glob_Point & p_2, Obj_Patch & output)
{
//	int width  = 0; // width of the object
//	int height = 0; // height of the object
	Glob_Point l_c_point;// the top left corner point 
//	Glob_Point lc_index: // top left corner index point in global map
	l_c_point[0]= p_1[0]<p_2[0]?p_1[0]:p_2[0];// left value

	l_c_point[1]= p_1[1]>p_2[1]?p_1[1]:p_2[1];// top value

	// tan_value
	output.tan_value = atan2((p_2[1]-p_1[1]),(p_2[0]-p_1[0]));  // be careful with the angle, the direction of h
    std::cout<<"tan_value"<<output.tan_value<<std::endl;
//----------------------------------get the corner information-----------------------------------
    if (l_c_point[0] < 0)
	l_c_point[0] = std::floor(l_c_point[0]/resolution);// floor the value of x point when it is negtive
	else
	l_c_point[0] = std::ceil(l_c_point[0]/resolution); // ceil the value when it is possitive
	if (l_c_point[1] < 0)
	l_c_point[1] = std::floor(l_c_point[1]/resolution);// floor the value of y point when it is negtive
	else
	l_c_point[1] = std::ceil(l_c_point[1]/resolution);// ceil the value when it is possitive

/*--------------------------------------------------------------------------*/
    // test
//	std::cout<< "width of the map"<< l_c_point[0]<<std::endl;
/*--------------------------------------------------------------------------*/
	     
    l_c_point[0] = Map_Size/2 + l_c_point[0]; // fit the Mat Map coordinate x axis
		
    l_c_point[1] = Map_Size/2 - l_c_point[1]; // fit the Mat Map coordinate y axis

    output.Corner_p[0] = l_c_point[1];   // put into output

    output.Corner_p[1] = l_c_point[0];   // put into output
//----------------------------------get the w h information------------------------------------
	output.width = std::ceil(std::abs(p_1[0] - p_2[0])/resolution);   // ceil the value
	output.height = std::ceil(std::abs(p_1[1] - p_2[1])/resolution);   // ceil the value
//----------------------------------------------------------------------------------------------
	
//	std::cout<< "width of the map"<< output.width<<std::endl;
}



/**********************************************************************************************/



// Build triangle occupancy map
void Tri_grid (Obj_Patch & input, float weight,cv::Mat & output)
{
	std::cout<<"tan_value"<< input.tan_value<<std::endl;
	std::cout<<"grid_value width"<< input.width<<" grid height"<<input.height<<std::endl;
    output = cv::Mat::zeros(input.height,input.width,CV_32FC1); // patch
//	double temp_v = 0;
	if (1.56>input.tan_value && input.tan_value>=0)//   first quadrant
	{
//		output.at<float>(0,input.width-1) = weight;//stupid...............
		for (unsigned int i = 0; i < input.height; i++)
		{
			for (unsigned int j = input.width; j >(input.width-std::ceil(((double)i+1)/input.height*input.width)) ; j--)
			{
				output.at<float>(i, j-1) += weight; 
			}
		}
	
	}


	else if (3.14>input.tan_value && input.tan_value >= 1.56)// second quadrant
	{

//		output.at<float>(input.height-1,input.width-1) = weight;
		for (unsigned int i = 0; i < input.height; i++)
		{
			for (unsigned int j = input.width; j > std::floor((double) i/input.height*input.width) ; j--)
			{
 				std::cout<<" float   "<< (double) 5/input.height*input.width<<std::endl;
            	output.at<float>(i, j-1) += weight; 
			}
		}
	
	}

//
	else if (-1.56>input.tan_value && input.tan_value >= -3.14)// fourth quadrant
	{
		for (unsigned int i = 0; i < input.height; i++)
		{

			for (unsigned int j = 0; j < (input.width-std::floor((double) i/input.height*input.width)) ; j++)
			{
				output.at<float>(i, j) += weight; 
			}
		}

	}

	else// fourth quadrant;
	{
		for (unsigned int i = 0; i < input.height; i++)
		{
			for (unsigned int j = 0; j < std::ceil(((double) i+1)/input.height*input.width) ; j++)
			{
				output.at<float>(i, j) += weight; 
			}
		}


	} 

}

void map_updating(cv::Mat & Map )
{
	
	int cols = Map.cols;
	int rows = Map.rows;

}

void map_thresholding(cv::Mat & Map, double threshold)
{

	for(unsigned int i = 0; i < Map.cols; i++)
	{	
		for(unsigned int j = 0; j < Map.rows; j++)
		{
			if(Map.at<float>(j,i) < threshold)
	 		Map.at<float> = 0;
		}
	}

}


int main(int argc, char* argv[])
{
    // the totall area of the map would be 5 meter by 5 meter
    raw_map =  cv::Mat::zeros(Map_Size,Map_Size,CV_32FC1); // 100 by 100 grid map with a resolution of 5cm.(Matlab style,I like it)
	raw_map.at<float>(Map_Size/2,Map_Size/2)= 10;
    
    // the origin is always at the center of the map. which is very easy to do the calculation

	std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > input_b_points;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	// the transformation is actually by using the function (100+x,100-y)    
    pcl::PCDReader reader;
	reader.read("c_3.pcd",*cloud);
    // GPS setting
    float g_x,g_z,g_angle;
	std::cout<< "please input the gps data :"<< std::endl;
	std::cin >> g_x >> g_z >> g_angle;
	gps.x = g_x;
	gps.z = g_z;
	gps.angle = g_angle;
    
	// transform the global coordinate
	
	Global_tran (glob_obj_points, cloud);
	std::cout<< " global points"<< glob_obj_points[0]<<std::endl;
    // get the patches of one object
	for( unsigned int i= 0; i< glob_obj_points.size()-1; i++)
	{
		Obj_Patch temp_patch;// one patch
	    Get_patch (glob_obj_points[i], glob_obj_points[i+1], temp_patch);
		obj_patches.push_back(temp_patch);

	}
	// build the triangle occupancy grid patch
	char buffer [50];
	for (unsigned int i= 0; i< obj_patches.size(); i++)
	{
		std::sprintf(buffer,"test%d",i);
		cv::Mat temp_1 = cv::Mat::zeros (obj_patches[i].height,obj_patches[i].width,CV_32FC1);
		
		std::cout<<"let us see111111"<<temp_1<<std::endl;	
		cv::Mat Temp (raw_map,cv::Rect(obj_patches[i].Corner_p[1],obj_patches[i].Corner_p[0],obj_patches[i].width,obj_patches[i].height));	
		std::cout<< "Rect" << Temp<<std::endl;
	//	Tri_grid (obj_patches[i], 10, Temp);
		Tri_grid (obj_patches[i], 10, temp_1);
		std::cout<<"let :us see"<<temp_1<<std::endl;	
		Temp += temp_1;    
//	cv::namedWindow(buffer,1);

//	cv::imshow(buffer, temp_1);
//    cv::waitKey(1000);
	}
	cv::namedWindow("global map",1);
  
	cv::imshow("global map", raw_map);
    cv::waitKey(0);
}
    
    

#endif      
