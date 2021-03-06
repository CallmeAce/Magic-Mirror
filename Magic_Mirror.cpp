#include "Magic_Mirror.h"     
/** Constructor
 **
 */

Magic_Mirror::Magic_Mirror () : viewer (new pcl::visualization::PCLVisualizer("Magic Mirror")),m_cloud_1(new pcl::PointCloud<PointT>),m_cloud_2(new pcl::PointCloud<PointT>) //constractor initialization
{ 
}

/** Deconstructor
 **
 */
Magic_Mirror::~Magic_Mirror ()
{
}



/** Callback function
 * input : a constant pointer point to the kinect buffer
 * output: point cloud 
   Get the datas from Kinect
 */
void Magic_Mirror::cloud_cb_(const pcl::PointCloud<PointT>::ConstPtr &cloud)
{  
	*m_cloud_1 = *cloud;
 m_cloud_3 = cloud->makeShared();
//	std::cout<<"hello"<<std::endl;
}

void Magic_Mirror::Map_updating ()
{
}

//TODO
void Magic_Mirror::Magic_Visualizer ()
{
}


/** run function
 ** dance dance dance
 */

void Magic_Mirror::run()
{

	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	
	boost::function<void (const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&Magic_Mirror::cloud_cb_, this, _1);
	interface->registerCallback (f);
	interface->start ();
         boost::this_thread::sleep (boost::posix_time::seconds (1));
	Magic_Processing magic;// initial the Magic_Processing
	Magic_Mapping m_mapping;

	magic.Set_ROI (-1,3);
    		// Add grids in the visualizer
	PointT flh_point;
	PointT slh_point;
	PointT flv_point;
	PointT slv_point;

	char str[30];
//	int v1(0);
//	viewer->creatViewPort(0,0,0.2,0.2,v1);//creat a viewport
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
			viewer->addLine<pcl::PointXYZ> (flh_point,slh_point,0,0,1,str);
			sprintf(str,"grid horizontal%03d",i);
			viewer->addLine<pcl::PointXYZ> (flv_point,slv_point,0,0,1,str);
	}

			viewer->addCoordinateSystem (1.0);

     		m_mapping.Map_Init(); // map initialization
	while(!viewer->wasStopped())
	{
		magic.Cloud_ROI (m_cloud_1,magic.m_cloud_roi);//set ROI
		magic.Octree_Compression (magic.m_cloud_roi,magic.m_cloud_downS,0.01,0.01,0.01);//down sample
		magic.Ransac_ground_plane (magic.m_cloud_downS,magic.gr_coefficients);// use Ransac algorithm to get the ground plane parameters
		magic.BK_Filters(magic.m_cloud_downS,magic.wall_coefficient, magic.m_cloud_filtered, 0.02);// remove ground points and wall plane points
		magic.Clustering (magic.m_cloud_filtered, magic.m_cloud_cluster_points, _Cluster_NUM);//clustering
		magic.Projection (magic.m_cloud_cluster_points);// projection
   		magic.Rotation (magic.m_proj_vector,magic.m_rot_proj_vector);// rotation
		magic.TwoD_Convex_Hull (magic.m_rot_proj_vector,magic.m_convex_vector); //convex data
//		magic.Sample_D_Hull (magic.m_convex_vector, magic.m_Samp_vector); // the result of Magic Processing
        std::cout<<"the thresho_counter:"<<m_mapping.thresh_counter<<std::endl;
		float threshold = 1;
		m_mapping.Map_Update(m_mapping.Raw_Map, magic.m_convex_vector, threshold);
		if(m_mapping.thresh_counter >4)
		{
		    float t_th = 3;
        	m_mapping.Map_Thresholding (m_mapping.Raw_Map,t_th);
			m_mapping.thresh_counter = 0; // zero the counter;
		}
			// Add pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr v_case;

//		pcl::PCDWriter writer;
//		writer.write("c_1.pcd", *magic.m_Samp_vector[0]);

		for(unsigned int i = 0;i<magic.m_cloud_cluster_points.size();i++)
		{
			//      viewer->addPointCloud<PointT>(magic.m_cloud_filtered,"t");
		    	std::stringstream s1;
			std::stringstream s2;
			std::stringstream s3;
			s1<<"v"<<i;
			s2<<"vv"<<i;
			s3<<"vvv"<<i;


			v_case = magic.m_cloud_cluster_points[i].makeShared();// actually it makes a copy of pointcloud, it should be deleted manually         
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(magic.m_rot_proj_vector[i], 0, 255, 0);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(magic.m_convex_vector[i], 255, 0, 0);
			viewer->addPointCloud<PointT>(v_case,s1.str());
			viewer->addPointCloud<PointT>(magic.m_convex_vector[i],single_color_1,s3.str());
		
        	}
		
  boost::this_thread::sleep (boost::posix_time::seconds (1));
// 			  viewer->resetCameraViewpoint("vv0");

	   // while(!viewer->wasStopped())
	   // {
 
	//		viewer->addPointCloud<PointT>(magic.m_cloud_roi,"1");
        //                std::cout<<"cloud grabber"<< m_cloud_3->points.size()<<std::endl;
       
	   		 viewer->spinOnce (100);
       //               viewer->removePointCloud("1");

//
//
		for(unsigned int i = 0;i<magic.m_cloud_cluster_points.size();i++)
        {
		 	std::stringstream s1;
			std::stringstream s2;
			std::stringstream s3;
			s1<<"v"<<i;
			s2<<"vv"<<i;
			s3<<"vvv"<<i;


 		    viewer->removePointCloud(s1.str());
        	    viewer->removePointCloud(s2.str());
	            viewer->removePointCloud(s3.str());
        }
  // 	}

	}

}












