#include "Magic_Mapping.h"

/* Constructor
 * 
 */
Magic_Mapping::Magic_Mapping () : Map_Size(1000)   // set the map size
{
	counter = 0;
	thresh_counter = 0;
	_gps.x = 0;//initial position , to be discussed
	_gps.z = 0;
	_gps.angle = 0;
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
/* Get Patch of the cluster
 * input: cluster points
 * output: Mat format patch
 */
void Magic_Mapping::Get_Patch ( std::vector<Glob_Point> & input_points, Obj_Patch_1 & output)
{
    Glob_Point l_c_point;// the top left corner point 
	Glob_Point r_c_point; // the bottom right corner point
//  Glob_Point lc_index: // top left corner index point in global map
	l_c_point = input_points[0];
	r_c_point = input_points[0];
	for(unsigned int i = 0; i < input_points.size(); i++)
	{
	    l_c_point[0]= l_c_point[0]<input_points[i][0]?l_c_point[0]:input_points[i][0];// left value

	    l_c_point[1]= l_c_point[1]>input_points[i][1]?l_c_point[1]:input_points[i][1];// top value

	    r_c_point[0]= r_c_point[0]>input_points[i][0]?r_c_point[0]:input_points[i][0];// right value
	    
		r_c_point[1]= r_c_point[1]<input_points[i][1]?r_c_point[1]:input_points[i][1];// bottom value

	}
	Glob_Point p_temp = l_c_point;	// for width calculation

/* test */
	std::cout<<"left corner point"<< l_c_point<<std::endl;
	std::cout<<"right bottom point"<< r_c_point<<std::endl;
/* test end */
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
	output.width = std::ceil(std::abs(p_temp[0] - r_c_point[0])/resolution);   // ceil the value
	output.height = std::ceil(std::abs(p_temp[1] -r_c_point[1])/resolution);   // ceil the value
	//----------------------------------------------------------------------------------------------
	std::cout<<" patch_1 width   :"<< output.width<<std::endl;
	std::cout<<" patch_1 height  :"<< output.height<<std::endl;
	if(output.width ==0)
			output.width =1;
	if(output.height ==0)
			output.height =1;

}


/* Triangle Occupancy Grid Map
 * 
 */

void Magic_Mapping::Tri_Grid (Obj_Patch & input, float & weight, cv::Mat & output)
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

/* transform the global points into indices

 */
void Magic_Mapping::P_Index(std::vector<Glob_Point>& input_points, std::vector<Eigen::Vector2f> & output)
{
	output.clear();// clear the output
	Eigen::Vector2f temp_p;// the top left corner point 
	Eigen::Vector2f temp_corner; // just for fit the coordinate swap the x, z

	for(unsigned int i = 0; i < input_points.size(); i++)
	{
		if (input_points[i][0] < 0)
			temp_p[0] = std::floor(input_points[i][0]/resolution);// floor the value of x point when it is negtive
		else
			temp_p[0] = std::ceil(input_points[i][0]/resolution); // ceil the value when it is possitive
		if (input_points[i][1] < 0)
			temp_p[1] = std::floor(input_points[i][1]/resolution);// floor the value of y point when it is negtive
		else
			temp_p[1] = std::ceil(input_points[i][1]/resolution);// ceil the value when it is possitive
			temp_p[0] = Map_Size/2 +temp_p[0]; // fit the Mat Map coordinate x axis

			temp_p[1] = Map_Size/2 -temp_p[1]; // fit the Mat Map coordinate y axis
				
			temp_corner[0]= temp_p[1];

			temp_corner[1]= temp_p[0];
			// test
			std::cout<< "indices points"<< temp_corner<<std::endl;
			// end test
			output.push_back(temp_corner);
 	}
}


/* Bresenham algorithm

 */

void Magic_Mapping::Bresenham_line(int x0, int  y0, int  x1, int  y1, float& weight, cv::Mat & output)
{

    int dx = abs(x1-x0);
    int dy = abs(y1-y0);  
	int sx;
	int sy;
	int err;
	int e_2;
    if (x0<x1)// differnt quadrant
    sx = 1;
    else
    sx = -1;
    if (y0<y1)
    sy = 1;
    else
    sy = -1;
    err = dx - dy;
//	int i = 0;// width
//    int j = 0;// height
    while(1)
	{
        output.at<float>(x0,y0) = weight;
        if (x0==x1 && y0 == y1)
        break;
        e_2 = 2*err;
        if(e_2> -dy)
        {
        	err  = err -dy;
         	x0   = x0 + sx;
        }
		
		else if (e_2<dx)
		{
			err = err + dx;
			y0 = y0 + sy;
		}
		
		
	}

}

// implementation of Bresenham's line algorithm
void Magic_Mapping::Bresenham (Obj_Patch & input, float & weight, cv::Mat & output)
{
//	int dx = input.width;
//	int dy = input.height;	
//	if (1.56>input.tan_value || input.tan_value>=-1.56)// differnt quadrant
//	int sx = 1;
//	else
//	int sx = -1;
//	if (1.56<=input.tan_value || input.tan_value<-1.56)
//	int sy = 1;
//	else
//	int sy = -1;
//	int err = dx - dy;
//	int j =0;
//	for(unsigned int i=0; i< input.width; i++)
//	{
//		output.at<float>(i,j) = weight;
//		if (i==input.width && j == input.height)
//		break;
//		float e_2 = 2*err;
//		if(e_2> -dy)
//		{
//			err = err -dy;
//			i   = i + sx;
//		}
//	}
    std::cout<<"tan_value"<< input.tan_value<<std::endl;
    std::cout<<"grid_value width"<< input.width<<" grid height"<<input.height<<std::endl;
    output = cv::Mat::zeros(input.height,input.width,CV_32FC1); // patch
//  double temp_v = 0;
    if (1.56>input.tan_value && input.tan_value>=0)//   first quadrant
    {
//      output.at<float>(0,input.width-1) = weight;//stupid...............
		Bresenham_line(input.height-1,0, 0, input.width-1,weight,output);
    }




    else if (3.14>input.tan_value && input.tan_value >= 1.56)// second quadrant
    {

//      output.at<float>(input.height-1,input.width-1) = weight;

		Bresenham_line(input.height-1,input.width-1, 0, 0, weight,output);

    }

    else if (-1.56>input.tan_value && input.tan_value >= -3.14)// third quadrant
    {

		Bresenham_line(0,input.width-1, input.height-1, 0, weight, output);

    }


    else// fourth quadrant;
    {


		Bresenham_line(0,0, input.height-1, input.width-1,weight, output);
    }
}

/* small point size judgement
 * for Patch Injection
 */
int Magic_Mapping::To_right (Eigen::Vector2f & p1, Eigen::Vector2f & p2, Eigen::Vector2f & p3)
{
	return((p2[0]-p1[0])*(p3[1]-p1[1]) - (p3[0]-p1[0])*(p2[1]-p1[1]));
}


/* Patch Injection  
 * the grids on the right side of the lines, will be occupied
 */

void Magic_Mapping::Patch_Injection (Obj_Patch_1 & cluster_patch, std::vector<Eigen::Vector2f> & input_patches, float & weight,cv::Mat & output)
{
	std::vector<Eigen::Vector2f> points_ind;
	for(unsigned int i = 0; i< input_patches.size(); i++)
	{
//	 	P_index p_ind;   // points indices
//	    p_ind.x = (int) cluster_patch.Corner_p[0];
//	    p_ind.y = (int) cluster_patch.Corner_p[1];
		Eigen::Vector2f p;
		p = input_patches[i] - cluster_patch.Corner_p;
		std::cout<< "corner points  "<< cluster_patch.Corner_p<<std::endl;
		points_ind.push_back (p);
		std::cout<< " the cluster patch indices : " <<std::endl;
		
		std::cout<< " x : " <<p[0]<<std::endl;
		std::cout<< " y : " <<p[1]<<std::endl;
	}	
//	Eigen::Vector2f p_f; //first point
//	p_f =  input_patches[0].Corner_p - cluster_patch.Corner_p;//first point

//	points_ind.push_back(p_f);//first point
	
	Eigen::Vector2f temp;// the point to be checked
	
		std::cout<<"cluster height  :"<< cluster_patch.height<<std::endl;
		std::cout<<"cluster width  :"<< cluster_patch.width<<std::endl;
	int ct;     //counter
	for(unsigned int i=0; i< cluster_patch.height; i++)// loop through the cluster patch
	{
		std::cout<<"hello points indices  :"<< points_ind.size()<<std::endl;
		for(unsigned int j=0; j< cluster_patch.width; j++)
		{	
			temp[0] = i;
			temp[1] = j;
			ct = 0;
			for(unsigned int k=0; k< points_ind.size()-1; k++)
			{
			//	std::cout<<"hello_1  :  "<<ct<<std::endl;
				if(To_right(points_ind[k],points_ind[k+1],temp)<=0)
				ct++;
				else
				break;
			}
			if(ct == points_ind.size()-1)
			{   
				std::cout<< "what?"<<std::endl;
				output.at<float>(temp[0],temp[1]) = weight;
			}
		}
	}

}  



/* Map_Run/ Map_update
 *
 */

void Magic_Mapping:: Map_Update (cv::Mat & input_Map, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > & input_points, float & weight )
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


/*--------------------------*/
	for(unsigned int i = 0; i < input_points.size(); i++)
	{

		Global_Transformation (_g_points_v,input_points[i]);// get the global points from cluster
		Get_Patch (_g_points_v,_patches_v);
		std::cout<<"patches size: "<<_patches_v.size()<<std::endl;
		P_Index(_g_points_v,_g_ind_v); // get indices vector
		Obj_Patch_1 cluster_patch; // the cluster patch
		Get_Patch (_g_points_v,cluster_patch);
		std::cout<<"cluster_patch's width: "<< cluster_patch.width<<std::endl;

		cv::Mat media_patch = cv::Mat::zeros (cluster_patch.height, cluster_patch.width,CV_32FC1);// media mat patch
    	cv::Mat Rect_Map (input_Map,cv::Rect(cluster_patch.Corner_p[1], cluster_patch.Corner_p[0], cluster_patch.width, cluster_patch.height));// A mat formate pointers point to the Global Map;
		for (unsigned int j =0; j< _patches_v.size(); j++)
		{
			cv::Mat temp_1 = cv::Mat::zeros (_patches_v[j].height, _patches_v[j].width,CV_32FC1);// media mat patch

    	    cv::Mat Temp (input_Map,cv::Rect(_patches_v[j].Corner_p[1], _patches_v[j].Corner_p[0], _patches_v[j].width, _patches_v[j].height));// A mat formate pointers point to the Global Map;


//TODO some changes here		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//        Bresenham (_patches_v[j], weight, temp_1);

	 //       Tri_Grid (_patches_v[j], weight, temp_1);
//TODO some change end here     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//	        std::cout<<"let :us see"<<temp_1<<std::endl;
	  //      Temp += temp_1;

		}
	
		Patch_Injection(cluster_patch, _g_ind_v, weight, media_patch);
		Rect_Map += media_patch; 	
	}
	if(counter ==0|counter==6)
	{
		std::cout << " please input the gps and gyro angle data (separate with space) :" << std::endl;
		std::cin >> g_cin_x >> g_cin_z >> g_cin_angle;

	    _gps.x = g_cin_x;
	    _gps.z = g_cin_z;
	    _gps.angle = g_cin_angle/180 * pi;
		counter =0;
	}


	cv::imshow("Hello Kitty", input_Map);// show the global map
	cv::waitKey(100);
    counter++;
	thresh_counter++;
}


/* Map thresholding
 * remove the noise from the global map
 */

void Magic_Mapping::Map_Thresholding (cv::Mat & Map, float & threshold)
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












