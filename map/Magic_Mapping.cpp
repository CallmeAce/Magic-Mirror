#include "Magic_Mapping.h"

/* Constructor
 * 
 */
Magic_Mapping::Magic_Mapping () : Map_Size(1000)   // set the map size
{
	counter = 0;
	thresh_counter = 0;
}

/* Disconstruction */
Magic_Mapping::~Magic_Mapping ()
{
}

/* Map initialization
 *
 */
void Magic_Mapping::Map_Init()
{
	Raw_Map = cv::Mat::zeros(Map_Size, Map_Size, CV_32FC1); // Build the square grid map according to the size defined

	Raw_Map.at<float>(Map_Size/2, Map_Size/2) = 20; // Mark the origin on the map
	
}



/* Global Transformation
 * 
 */
void Magic_Mapping::Global_Transformation (std::vector<Glob_Point> & output, pcl::PointCloud<PointT>::Ptr & input_point)
{
	output.clear(); // clear the vector
	    // rotation matrix around y axis
    G_R_Matrix r_matrix;
    r_matrix <<  cos(-_gps.angle), sin(-_gps.angle),
                -sin(-_gps.angle), cos(-_gps.angle);
    // transition vector
    Glob_Point trans_point;
    trans_point << _gps.x, _gps.z;
    // put x, z data into the Glob point structure 
    Glob_Point temp_point;
    // push_back the points
    for(unsigned int i = 0; i< input_point->points.size(); ++i)
    {
        temp_point << input_point->points[i].x,
        input_point->points[i].z;

//      temp_point = temp_point + trans_point;  
        temp_point = r_matrix * temp_point;
        temp_point = temp_point + trans_point;
        output.push_back(temp_point);
    }

}

/* Get the Bounding box patch
 *
 */
void Magic_Mapping::Get_Patch (Glob_Point & p_1, Glob_Point & p_2, Obj_Patch & output)
{
    Glob_Point l_c_point;// the top left corner point 
//  Glob_Point lc_index: // top left corner index point in global map
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
    l_c_point[0] = Map_Size/2 + l_c_point[0]; // fit the Mat Map coordinate x axis

    l_c_point[1] = Map_Size/2 - l_c_point[1]; // fit the Mat Map coordinate y axis

    output.Corner_p[0] = l_c_point[1];   // put into output

    output.Corner_p[1] = l_c_point[0];   // put into output
//----------------------------------get the w h information------------------------------------
    output.width = std::ceil(std::abs(p_1[0] - p_2[0])/resolution);   // ceil the value
    output.height = std::ceil(std::abs(p_1[1] - p_2[1])/resolution);   // ceil the value
//----------------------------------------------------------------------------------------------
	if(output.width ==0)
	output.width =1;
	if(output.height ==0)
	output.height =1;
}

/* function overloading */

void Magic_Mapping::Get_Patch (std::vector<Glob_Point> & input_points, std::vector<Obj_Patch,Eigen::aligned_allocator<Obj_Patch> > & output_points)
{
	output_points.clear();// clear the Obj container
	for(unsigned int i = 0; i < input_points.size()-1; i++)
	{
		Obj_Patch temp_patch;
		Get_Patch (input_points[i], input_points[i+1], temp_patch);
	    std::cout<<"strange"<<temp_patch.tan_value<<std::endl;	
		output_points.push_back (temp_patch);
	}

}

/* Triangle Occupancy Grid Map
 * 
 */

void Magic_Mapping::Tri_Grid (Obj_Patch & input, float weight, cv::Mat & output)
{
    std::cout<<"tan_value"<< input.tan_value<<std::endl;
    std::cout<<"grid_value width"<< input.width<<" grid height"<<input.height<<std::endl;
    output = cv::Mat::zeros(input.height,input.width,CV_32FC1); // patch
//  double temp_v = 0;
    if (1.56>input.tan_value && input.tan_value>=0)//   first quadrant
    {
//      output.at<float>(0,input.width-1) = weight;//stupid...............
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

//      output.at<float>(input.height-1,input.width-1) = weight;
        for (unsigned int i = 0; i < input.height; i++)
        {
            for (unsigned int j = input.width; j > std::floor((double) i/input.height*input.width) ; j--)
            {
        //        std::cout<<" float   "<< (double) 5/input.height*input.width<<std::endl;
                output.at<float>(i, j-1) += weight;
            }
        }

    }


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

/* Map_Run/ Map_update
 *
 */

void Magic_Mapping:: Map_Update (cv::Mat & input_Map, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > input_points, float weight )
{

	cv::namedWindow("Hello Kitty",1); // 
/* Manually set the GPS data */
 
	float g_cin_x;// input x
	float g_cin_z;// input z
	float g_cin_angle;// input angle
//	float g_cin_x;// input x
//	float g_cin_z;// input z
//	float g_cin_angle;// input angle
//	std::cout << " please input the gps and gyro angle data (separate with space) :" << std::endl;
//	std::cin >> g_cin_x >> g_cin_z >> g_cin_angle;

//	_gps.x = 0;
//	_gps.z = 0;
//	_gps.angle = 0;

/*  save the map */

	char name [50];

	std::sprintf(name,"map/map%03d.ppm",counter);
	cv::imwrite(name,Raw_Map);
		if(counter ==0|counter==7)
		{
			std::cout << " please input the gps and gyro angle data (separate with space) :" << std::endl;
			std::cin >> g_cin_x >> g_cin_z >> g_cin_angle;

		    _gps.x = g_cin_x;
		    _gps.z = g_cin_z;
		    _gps.angle = g_cin_angle/180 * pi;
			counter =0;
		}


/*--------------------------*/
	for(unsigned int i = 0; i < input_points.size(); i++)
	{
	//	if(thresh_counter ==0)
	//	{
	//		std::cout << " please input the gps and gyro angle data (separate with space) :" << std::endl;
	//		std::cin >> g_cin_x >> g_cin_z >> g_cin_angle;

	//	    _gps.x = g_cin_x;
	//	    _gps.z = g_cin_z;
	//	    _gps.angle = g_cin_angle/180 * pi;

	//	}
		Global_Transformation (_g_points_v,input_points[i]);// get the global points from cluster
		Get_Patch (_g_points_v,_patches_v);
//		char name [50];
		std::cout<<"patches size: "<<_patches_v.size()<<std::endl;
		for (unsigned int j =0; j< _patches_v.size(); j++)
		{
		//	std::sprintf(name,"map/map%03d.ppm",j);
			cv::Mat temp_1 = cv::Mat::zeros (_patches_v[j].height, _patches_v[j].width,CV_32FC1);// media mat patch

    	    cv::Mat Temp (input_Map,cv::Rect(_patches_v[j].Corner_p[1], _patches_v[j].Corner_p[0], _patches_v[j].width, _patches_v[j].height));// A mat formate pointers point to the Global Map;
 //   	    std::cout<< "Rect" << Temp<<std::endl;
    //  Tri_grid (obj_patches[i], 10, Temp);
	        Tri_Grid (_patches_v[j], weight, temp_1);
	        std::cout<<"let :us see"<<temp_1<<std::endl;
	        Temp += temp_1;
			
//            std::cout<<"error ?"<<std::endl;				

		}

	}
	cv::imshow("Hello Kitty", input_Map);// show the global map
	cv::waitKey(100);
    counter++;
	thresh_counter++;
}


/* Map thresholding
 * remove the noise from the global map
 */

void Magic_Mapping::Map_Thresholding (cv::Mat & Map, float threshold)
{

    for(unsigned int i = 0; i < Map.cols; i++)
    {
        for(unsigned int j = 0; j < Map.rows; j++)
        {
            if(Map.at<float>(j,i) < threshold)
            Map.at<float>(j,i) =0;
        }
    }

}







