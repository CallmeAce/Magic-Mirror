#include "Magic_Processing.h"
/** constructor
 *

 */
Magic_Processing::Magic_Processing(): m_cloud_downS (new pcl::PointCloud<PointT>),m_cloud_projection(new pcl::PointCloud<PointT>),m_cloud_roi(new pcl::PointCloud<PointT>),m_cloud_filtered(new pcl::PointCloud<PointT>),m_cloud_convex(new pcl::PointCloud<PointT>),m_cloud_rotation(new pcl::PointCloud<PointT>),ground_coefficients(4),wall_coefficient(4),gr_coefficients(4){}

/**

 *Change the private Roi parameter

 */
void Magic_Processing::Set_ROI (float para_y, float para_z)
{
     _Roi_z = para_z;
     _Roi_y = para_y;	

}

/**

 *Get the pointcloud within the range user defined 

 */
void Magic_Processing::Cloud_ROI (pcl::PointCloud<PointT>::Ptr & inputcloud,pcl::PointCloud<PointT>::Ptr & outputcloud)
{
//  z axis 
    pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> z_pass;
    z_pass.setInputCloud (inputcloud);
    z_pass.setFilterFieldName("z");
	z_pass.setFilterLimits (0,_Roi_z);
    z_pass.filter (* temp_cloud);
    std::cout<<"what"<<_Roi_z<<std::endl;
    std::cout<< temp_cloud->points.size()<<std::endl;
// then y axis
    pcl::PassThrough<PointT> y_pass;
    y_pass.setInputCloud (temp_cloud);
    y_pass.setFilterFieldName ("y");
    y_pass.setFilterLimits (_Roi_y,0.5);
    y_pass.filter (*outputcloud);
    std::cout<< outputcloud->points.size()<<std::endl;
}
//  overload of Cloud_ROI
void Magic_Processing::Cloud_ROI (pcl::PointCloud<PointT>::Ptr & inputcloud)
{
//  clear the m_cloud_roi
    m_cloud_roi->clear();
//  z axis 
    pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> z_pass;
    z_pass.setInputCloud (inputcloud);
    z_pass.setFilterFieldName("z");
	z_pass.setFilterLimits (0.4,_Roi_z);
    z_pass.filter (* temp_cloud);
// then y axis
    pcl::PassThrough<PointT> y_pass;
    y_pass.setInputCloud (temp_cloud);
    y_pass.setFilterFieldName ("y");
    y_pass.setFilterLimits (_Roi_y,0.5);
    y_pass.filter (*m_cloud_roi);
}






/**

 * Octree voxelgrid method to sample down the points clouds

 */
void Magic_Processing::Octree_Compression (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud, float size_x, float size_y, float size_z)
{

	pcl::VoxelGrid<pcl::PointXYZ> vg; // build the voxel grid
    vg.setInputCloud (inputcloud);
    vg.setLeafSize (size_x, size_y, size_z);
    vg.filter (*outputcloud);
    std::cout << "PointCloud after downsampleing has" << outputcloud->points.size()<<"data points." <<std::endl;
}
// overload of Octree_Compression
void Magic_Processing::Octree_Compression (pcl::PointCloud<PointT>::Ptr & inputcloud, float size_x, float size_y, float size_z)
{
	pcl::VoxelGrid<pcl::PointXYZ> vg; // build the voxel grid
    vg.setInputCloud (inputcloud);
    vg.setLeafSize (size_x, size_y, size_z);
    vg.filter (*m_cloud_downS);
    std::cout << "PointCloud after downsampleing has" << m_cloud_downS->points.size()<<"data points." <<std::endl;
}



/**
 * use Ransac algorithm get the groundplane parameter and remove the points belong to the plane
 */

void Magic_Processing::Ransac_ground_plane (pcl::PointCloud<PointT>::Ptr & inputcloud, Eigen::VectorXf & outputcoefficient)
{
	// limit the range scope of the pointclouds then run the Ransac algorithm
    Set_ROI (-0.2,2.5);
    pcl::PointCloud<PointT>::Ptr temp_output (new pcl::PointCloud<PointT>);
	Cloud_ROI (inputcloud, temp_output);
  	if(temp_output->points.size()>300)
    {
		pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      	pcl::PointCloud<PointT>::Ptr plane_points (new pcl::PointCloud<PointT>);
//		pcl::PCDWriter writer;
 		seg.setOptimizeCoefficients(true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setRadiusLimits(0.5,2);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);
			// after interating 80 times, find the planar that has the most inliers
		seg.setInputCloud (temp_output);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () < 1)
		std::cout<< "couldn't estimate a planar model for the given pointcloud"<< std::endl;
        g_coefficients = coefficients;
		std::cout<<"waht" << coefficients->values[0]<<std::endl;
        ground_coefficients[0] = coefficients->values[0];// private parameter
        ground_coefficients[1] = coefficients->values[1];
        ground_coefficients[2] = coefficients->values[2];
        ground_coefficients[3] = coefficients->values[3];

        outputcoefficient[0] = coefficients->values[0];
        outputcoefficient[1] = coefficients->values[1];
        outputcoefficient[2] = coefficients->values[2];
        outputcoefficient[3] = coefficients->values[3];
            // extract the planar inliers from the input cloud
		    // pcl::ExtractIndices<pcl::PointXYZ> extract;
			// extract.setInputCloud (temp_output);
       		// extract.setIndices (inliers);
			// extract.setNegative (inliers);
	}	


} 

/**

 * Filter for filter out the wall plane and ground plane
 * input: inputcloud, the wall plane parameter, the threshold value define the extracing accuracy
 * output: outputcloud
 

 */


void Magic_Processing::BK_Filters (pcl::PointCloud<PointT>::Ptr & inputcloud,Eigen::VectorXf & wall_coefficient, pcl::PointCloud<PointT>::Ptr & outputcloud, float threshold)
{
	outputcloud->clear();
	if (wall_coefficient.size() != 4)
	std::cout<<"Invalid number of model coefficients "<<wall_coefficient.size()<<std::endl;
	// Iterate through the 3d points and calculate the distances to the plane
    std::cout<< "filter"<<std::endl;
    for (unsigned int i = 0; i < inputcloud->points.size(); ++i)
	{
		Eigen::Vector4f pt (inputcloud->points[i].x,
                            inputcloud->points[i].y,
                            inputcloud->points[i].z,
 							1);
		if (std::fabs (ground_coefficients.dot (pt)) > threshold /*&& std::fabs(wall_coefficient.dot(pt))>threshold*/)
	    {
	    	outputcloud->points.push_back(inputcloud->points[i]);			
 		}
	}

}

/* Clustering
 * KNN clusteing method, choose the biggest three clusters
 * input: inputcloud, number of clusters user want to get
 * output: the output cluster pointclouds; the output cluster indices
 */
void Magic_Processing::Clustering(pcl::PointCloud<PointT>::Ptr & inputcloud, std::vector<pcl::PointCloud<PointT>,Eigen::aligned_allocator<pcl::PointCloud<PointT> > > & outputcloud, int num_cluster)
//void Magic_Processing::Clustering(pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT> & outputcloud, int num_cluster)
{
//	outputcloud.erase(outputcloud.begin(),outputcloud.end());
	m_cloud_cluster_indices.clear();//  my 3 hours!!!!!
    outputcloud.clear();

	if(inputcloud->points.size()>10)
	{
    //    m_cloud_cluster_indices.clear();//  my 3 hours!!!!!
    //    outputcloud.clear();
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud(inputcloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction <PointT> ec;
        ec.setClusterTolerance( 0.02); // 2cm
        ec.setMinClusterSize (300); // in other word, if the object has points less than 300, the robot can not see any thing. just for test
        ec.setMaxClusterSize (25000); // incase the object is too big.....
        ec.setSearchMethod (tree);
        ec.setInputCloud (inputcloud);
        ec.extract (m_cloud_cluster_indices);
            cluster_indices = m_cloud_cluster_indices;
        unsigned int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
                    std::cout<< "cluster"<<std::endl; 
        	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        	for( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        	{	
        		cloud_cluster->points.push_back(inputcloud->points[*pit]);
               // outputcloud[1].points.push_back(inputcloud->points[*pit]);
        	}    
            std::cout<< "cluster number......"<<cloud_cluster->points.size()<<std::endl;
//      	cloud_cluster->width = cloud_cluster->points.size();
//      	cloud_cluster->height = 1;
//      	cloud_cluster->is_dense= true;
//      	outputcloud[j].swap( *cloud_cluster);			//select the biggest 3 clusters
                    outputcloud.push_back(*cloud_cluster);
            if (j==num_cluster-1)
        	break;
        	j++;

        }
	}
}
/** Project the object points to the ground plane
 * input: inputcloud
 * output: outputcloud
 */



void Magic_Processing::Projection(pcl::PointCloud<PointT>::Ptr & inputcloud,pcl::PointCloud<PointT>::Ptr & outputcloud)
{
	
	if(inputcloud->points.size()>0)
	{
    	pcl::ProjectInliers<PointT> project;
    	project.setModelType (pcl::SACMODEL_PLANE);
    	project.setInputCloud (inputcloud);
    	project.setModelCoefficients (pg_coefficients);
    	project.filter (*outputcloud);
    	std::cerr<<"pointcloud after projection has"<< outputcloud->points.size()<<"data points."<<std::endl;
    }

}
//overloading
void Magic_Processing::Projection(pcl::PointCloud<PointT>::Ptr & inputcloud,pcl::PointCloud<PointT>::Ptr & outputcloud, pcl::ModelCoefficients::Ptr coefficients)
{
 	outputcloud->clear(); 
	if(inputcloud->points.size()>0)
	{
    	pcl::ProjectInliers<PointT> project;
    	project.setModelType (pcl::SACMODEL_PLANE);
    	project.setInputCloud (inputcloud);
    	project.setModelCoefficients (coefficients);
    	project.filter (*outputcloud);
    	std::cerr<<"pointcloud after projection has"<< outputcloud->points.size()<<"data points."<<std::endl;
    }
}
//overloading
void Magic_Processing::Projection(std::vector<pcl::PointCloud<PointT>,Eigen::aligned_allocator<pcl::PointCloud<PointT> > > & inputcloud)
{
    
    m_proj_vector.clear();//empty the vector
	if(inputcloud.size()>0)
	{
       // m_proj_vector.clear();//empty the vector
         //   pcl::PointCloud<PointT>::Ptr temp_incloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr temp_incloud;
        pcl::PointCloud<PointT>::Ptr temp_outcloud (new pcl::PointCloud<PointT>);
    	pcl::ProjectInliers<PointT> project;
    	project.setModelType (pcl::SACMODEL_PLANE);
    	project.setModelCoefficients (g_coefficients);
            for(unsigned int i = 0; i < inputcloud.size(); i++)
    	{ 
    	  //      temp_incloud->swap(inputcloud[i]);
                    temp_incloud = inputcloud[i].makeShared();
            	project.setInputCloud (temp_incloud);
    		project.filter (*temp_outcloud);
            
    		std::cerr<<"pointcloud after projection has"<< temp_outcloud->points.size()<<"data points."<<std::endl;
    
    		m_proj_vector.push_back(temp_outcloud->makeShared());// merit of smartpointer
    	}
	}
}



/**rotation points according to ground plane
 * input: inputcloud
 * output: outputcloud
 */

void Magic_Processing::Rotation(pcl::PointCloud<PointT>::Ptr & inputcloud,pcl::PointCloud<PointT>::Ptr & outputcloud)
{
	
    outputcloud->clear();// empty the output
	if(inputcloud->points.size()>0)
	{
   //     outputcloud->clear();// empty the output
    	// compute angle 
    	float a_z = -atan2( ground_coefficients[0],-ground_coefficients[1]); // rotation around z axis   
    
    	float a_y = atan2( ground_coefficients[2],ground_coefficients[0]); // rotation around y axis   
    
    	float a_x = -atan2( ground_coefficients[2],-ground_coefficients[1]); // rotation around z axis  
    	
    	Eigen::Matrix3f rot_Matrix_z; // rotation matrix_z
    
    	Eigen::Matrix3f rot_Matrix_y; // rotation matrix_y
    
    	Eigen::Matrix3f rot_Matrix_x; // rotationn matrix_x
    
    	Eigen::Matrix3f Rot_Matrix; //   main rotation matrix
    
    	rot_Matrix_z <<           cos(a_z), -sin(a_z),  0,
                 				  sin(a_z),  cos(a_z),  0,
    	    	        		  0,         0,         1;
    
    	rot_Matrix_y <<           cos(a_y),  0,    sin(a_y),
    				              0,         1,           0,
    			             	 -sin(a_y),  0,    cos(a_y);
    
    	rot_Matrix_x <<           1,         0,           0,
    				         	  0,  cos(a_x),   -sin(a_x),
    					          0,  sin(a_x),    cos(a_x);
    
    	Rot_Matrix  = rot_Matrix_z  * rot_Matrix_x;
    
    // move all the points along the direction of the normal of the plane with "d_t_0", where d is the distance from original point (0,0,0) to the plane
    
        float d_t_0 = ground_coefficients[3];
    
        Eigen::Vector3f point;
        Eigen::Vector3f trans_d;
    
       //normalize the plane norm
    //-------------------------------------------------------
    	Eigen::Vector3f trans_dd;
    	Eigen::Vector4f coefficient_norm;
    	coefficient_norm = ground_coefficients;
    	float sq_sum_1 = coefficient_norm.norm();
    	std::cout<< "norm is "<< sq_sum_1<<std::endl;
    	//--------------------------------------------------------
    	Eigen::Vector3f coefficient_norm_1;
    	float sq_sum  = sqrt(ground_coefficients[0]*ground_coefficients[0]+ground_coefficients[1]*ground_coefficients[1]+ground_coefficients[2]*ground_coefficients[2]);// normalize the vector
    	coefficient_norm_1[0] = ground_coefficients[0]/sq_sum;
    	coefficient_norm_1[1] = ground_coefficients[1]/sq_sum;
    	coefficient_norm_1[2] = ground_coefficients[2]/sq_sum;
    	d_t_0                 = ground_coefficients[3]/sq_sum;//including the distance
    	trans_d               = coefficient_norm_1 * d_t_0;
    
    	// point container
    	pcl::PointXYZ temp_point;
        pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
    	for(unsigned int i = 0; i< inputcloud->points.size();i ++)
    	{
    		point << inputcloud->points[i].x,
    				 inputcloud->points[i].y,
    			     inputcloud->points[i].z;
    
    		point = Rot_Matrix * point;
    
    		point = point + trans_d;
    
    		temp_point.x = point[0];
    		temp_point.y = point[1];
    		temp_point.z = point[2];
    
    		outputcloud->points.push_back(temp_point);
    
    	}
    
        
    // *m_cloud_rotation = *outputcloud; // make some copy
	}

}
//overloading
void Magic_Processing::Rotation(std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector )
{
	output_vector.clear();
	if(input_vector.size()>0)
	{
    //	output_vector.clear();
    	pcl::PointCloud<PointT>::Ptr tempcloud (new pcl::PointCloud<PointT>);
    	for (unsigned int i = 0; i < input_vector.size(); i++)
    	{
    		Rotation(input_vector[i],tempcloud);
            output_vector.push_back(tempcloud->makeShared());		
    	}	
	}
}

/** build the convex_hull from the input cloud
 *  input: input cloud
 *  output: output cloud
 */

void Magic_Processing::TwoD_Convex_Hull (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud)
{
	outputcloud->clear();
	if(inputcloud->points.size()>0)
	{
    //	outputcloud->clear();
        
    	Conv_Conc c_vex;	
    	c_vex.convex (inputcloud,outputcloud);
	}
}

  
void Magic_Processing::TwoD_Convex_Hull (std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector )
{
//	pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
	
    output_vector.clear();
	if(input_vector.size()>0)
	{	
    //	output_vector.clear();
        pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
    	for(unsigned int i = 0; i< input_vector.size(); i++)
    	{
    //		Conv_Conc *c_vex (new Conv_Conc);
    //		c_vex->convex (input_vector[i],c_vex->convex_hull);
    //		output_vector.push_back(c_vex->convex_hull->makeShared());
    //      delete c_vex;
        	TwoD_Convex_Hull (input_vector[i],temp_cloud);
    		output_vector.push_back(temp_cloud->makeShared());	
    
        }
	}
}

/** Sample down the convex hull pointcloud
 *  input: input cloud or input cloud vector
 *  output: output cloud or output cloud vector
 */

void Magic_Processing::Sample_D_Hull (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud)
{
	
    outputcloud->clear();
	if(inputcloud->points.size()>0)
	{
  //      outputcloud->clear();
    	SampleDown_con s_d;
    	s_d.SampleDown(inputcloud,outputcloud);
	}
}
// member function  overloading
void Magic_Processing::Sample_D_Hull (std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector )
{
   // pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
	
	output_vector.clear(); 
	if(input_vector.size()>0)
	{
	//    output_vector.clear(); 
		pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
		for(unsigned int i = 0; i < input_vector.size(); i++) 
		{
	
	//    	SampleDown_con *s_d(new SampleDown_con);
	//		s_d->SampleDown(input_vector[i],s_d->Sample_hull);
	//        output_vector.push_back(s_d->Sample_hull->makeShared());
	//        delete s_d;
			Sample_D_Hull (input_vector[i], temp_cloud);
			output_vector.push_back (temp_cloud->makeShared());
	
	  	}
	}
}

/**
 * To see weather the pointcloud can fit into the plane model
 */
bool Magic_Processing::is_wall(pcl::PointCloud<PointT>::Ptr & inputcloud)

{


}














