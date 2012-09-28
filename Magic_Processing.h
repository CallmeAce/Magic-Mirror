#ifndef MAGIC_PROCESSING_H
#define MAGIC_PROCESSING_H
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/concave_hull.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pcl/filters/project_inliers.h>
#include <utility>
#include <stack>


class Magic_Processing
{
	private:		 
     		// parameters setting
		    float _Roi_z; //default range distance
            float _Roi_y; //default height distance
     
            Eigen::VectorXf ground_coefficients; // ground plane coefficients
            pcl::ModelCoefficients::Ptr g_coefficients;// plane coefficients;
            pcl::ModelCoefficients::Ptr pg_coefficients;// ground plane model;
	public :
           
            typedef pcl::PointXYZ PointT; //define the point type
			pcl::PointCloud<PointT>::Ptr m_cloud_downS; //output of octree compression            
            pcl::PointCloud<PointT>::Ptr m_cloud_roi; // points of interest 2 meters distance and 0.5 meter height 
            Eigen::VectorXf wall_coefficient; // wall plane coefficients
 			Eigen::VectorXf gr_coefficients;
            pcl::PointCloud<PointT>::Ptr m_cloud_filtered; // remaining points after remval of points of ground and wall
            pcl::PointCloud<PointT>::Ptr m_cloud_projection; // points after projection


            int m_cluster_num;// define the number of clusters
            pcl::PointCloud<PointT>::Ptr m_cloud_convex;  // convex points 
            pcl::PointCloud<PointT>::Ptr m_cloud_concave;  // concave points 
            pcl::PointCloud<PointT>::Ptr m_cloud_rotation; // points after rotation adjustments;
            std::vector<pcl::PointIndices> m_cloud_cluster_indices; // cluster indices
      	    std::vector<pcl::PointCloud<PointT>,Eigen::aligned_allocator<pcl::PointCloud<PointT> > > m_cloud_cluster_points; //cluster points
       
      	    std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > >  m_proj_vector; //cluster projection points
       
      	    std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > >  m_rot_proj_vector; //cluster projection points
      	    std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > >  m_convex_vector; //convex points

      	    std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > >  m_Samp_vector; // convex points after being sampled down
      	//    pcl::PointCloud<PointT> m_cloud_cluster_points; //cluster points
      	//    pcl::PointCloud<PointT> m_cloud_cluster_points; //cluster points
            Magic_Processing(); //constractor                         
			



// Methods declaration      


         
           
		    // Change the ROI parameter                  
			void Set_ROI (float para_y,float para_z);
            // Set the ROI
            void Cloud_ROI (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud);
            void Cloud_ROI (pcl::PointCloud<PointT>::Ptr & inputcloud);
            // Octree algorithm compress the data
            void Octree_Compression (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud, float size_x, float size_y, float size_z);
            void Octree_Compression (pcl::PointCloud<PointT>::Ptr & inputcloud, float size_x, float size_y, float size_z);
           
            // Ransac algorithm
            void Ransac_ground_plane (pcl::PointCloud<PointT>::Ptr & inputcloud,Eigen::VectorXf & outputcoefficient);
            // Remove all the ground plane points as well as the wall points; utput the remaining points and the plane parameter
            void BK_Filters (pcl::PointCloud<PointT>::Ptr & inputcloud, Eigen::VectorXf & wall_coefficient, pcl::PointCloud<PointT>::Ptr & outputcloud, float threshold);            
            // Clustering the pointclouds and choose the biggest three clusters
            void Clustering (pcl::PointCloud<PointT>::Ptr & inputcloud, std::vector<pcl::PointCloud<PointT>,Eigen::aligned_allocator<pcl::PointCloud<PointT> > > & outputcloud, int num_cluster);
           // void Clustering (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT> & outputcloud, int num_cluster);
        	// Projection points on the ground plane   
        
		    void Projection (std::vector<pcl::PointCloud<PointT>,Eigen::aligned_allocator<pcl::PointCloud<PointT> > > & inputcloud);
	   	    void Projection (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud);
            void Projection (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud, pcl::ModelCoefficients::Ptr coefficients);
            // rotate the points clouds according to certain plane
            void Rotation (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud);

			void Rotation(std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector );
			// Compute the convex hull 
            void TwoD_Convex_Hull (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud);           

			void TwoD_Convex_Hull (std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector );

			// sample down the conc_conv hull 
            void Sample_D_Hull (pcl::PointCloud<PointT>::Ptr & inputcloud, pcl::PointCloud<PointT>::Ptr & outputcloud);           

			void Sample_D_Hull (std::vector<pcl::PointCloud<PointT>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & input_vector, std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr > > & output_vector );

			// wall or not
			bool is_wall(pcl::PointCloud<PointT>::Ptr & inputcloud);


};

						

/*----------------------------------------------------------------------------------------------------*/




/*---------------------------concave convex hull algorithms-------------------------------------------*/



class Conv_Conc

{
	public:
//--------------------------------Variable anouncement-----------------------------------------------
	typedef pcl::PointXYZ PointT;

	//container of the convex hull
	pcl::PointCloud<PointT>::Ptr convex_hull;

	//container of the concave hull
	pcl::PointCloud<PointT>::Ptr concave_hull;

	std::vector<std::pair<double,int> > radians_all;
	std::vector<std::pair<double,int> > x_list_pair;
	//std::vector<double> z_list;
	std::vector<std::pair<double,int> > z_list_pair;
	std::vector<std::pair<double,int> >::iterator it;
	//std::vector<comp_point> point_list;
	pcl::PointCloud<PointT>::iterator itt;
	pcl::PointCloud<PointT>::iterator itt1;
	// stack for graham
	std::stack<PointT,std::vector<PointT> > p_stack;
	std::vector<std::pair<double,int> > pp_dist_list;
	std::vector<std::pair<double,int> > pl_dist_list;
	static const double N = 4;  // threshold


// constructor
//	Conv_Conc () : convex_hull(new pcl::PointCloud<PointT>),concave_hull(new pcl::PointCloud<PointT>){};

	Conv_Conc () : convex_hull(new pcl::PointCloud<PointT>){};
//    Conv_Conc () ;


  //-------------------------------------------------------------------------


  //-----------------Function anouncement and defination-----------------------
  // judgement of on which side of line the point actualy lie
  // two points define a line. This is for computing the convex hull
	double To_left (PointT & c, std::stack<PointT,std::vector<PointT> > & pstack)
	{
    	PointT b  = pstack.top();
  	  pstack.pop();
    PointT a = pstack.top();
    pstack.push(b);
    return((b.x - a.x) * (c.z - a.z) - (c.x - a.x) * (b.z - a.z));

  };
  //-------------------Side judgement2---------------------------------------    
  double To_right (PointT & a, PointT & b, PointT & c)
  {

    return((b.x -a.x) * (c.z -a.z) - (c.x -a.x) * (b.z -a.z));

  };

 //--------------------Max function (return index)---------------------------
  int max_index( std::vector<std::pair<double,int> > &pair_list)
  {
   int size = pair_list.size();
   std::vector<std::pair<double, int> >::iterator iter;
   std::vector<std::pair<double, int> >::iterator largest_adr = pair_list.begin();
   for (iter = pair_list.begin();iter!=pair_list.end();iter++)
     if (largest_adr->first<iter->first)    // or: if (comp(*largest,*first)) for the comp version
     largest_adr=iter;
     size_t la = largest_adr - pair_list.begin();
   return (la);
  };


  //-------------------- Min function (return index)--------------------------
  int  min_index( std::vector<std::pair<double,int> > pair_list)
  {
    int size = pair_list.size();
    std::vector<std::pair<double,int> >::iterator iter;
    std::vector<std::pair<double,int> >::iterator smallest_adr = pair_list.begin();
    for (iter = pair_list.begin();iter!=pair_list.end();iter++)
    if (smallest_adr->first>iter->first)
    smallest_adr=iter;
    size_t la = smallest_adr - pair_list.begin();
    return (la);
  };

  //--------------------Pair sorting function---------------------------------
  struct Sortfunction
  {
    bool operator() (std::pair<double,int> a,std::pair<double,int> b)
    {
      return(a.first<b.first);
    };
  }sortfunction;


 //------------------------distance calculation---------------------------------
  //-----------------------Point to Point distance----------------------------
  double dist_pp_f (PointT &a, PointT &b)
  {
    double  diff_x   =  a.x - b.x;
    double  diff_z   =  a.z - b.z;
    return ( sqrt(diff_x*diff_x + diff_z*diff_z));
  };
  //---------------------point to line distance-------------------------
  //--------------------use heron's formula----------------------------
  double dist_pl_f (PointT &p1, PointT &p2, PointT &p3)
  {
    double a = dist_pp_f(p1,p2);
    double b = dist_pp_f(p1,p3);
    double c = dist_pp_f(p2,p3);
    if (b*b>=a*a+c*c)
    return (100);
    if (c*c>=a*a + b*b)
    return (100);
    double p = (a+b+c)/2;
    double s = sqrt(p*(p-a)*(p-b)*(p-c));

    return( 2*s/a);

  };




 	void convex (pcl::PointCloud<PointT>::Ptr & cloud, pcl::PointCloud<PointT>::Ptr & output_points)
	{
   		//prepare the datas
        output_points->clear();
    	int n      = cloud->points.size();
		for (int i = 0;i<cloud->points.size();i++)
		{
			std::pair <double,int> temp_xpair(cloud->points[i].x,i);// make x element pair;
        	std::pair <double,int> temp_zpair;// make z element pair;

			//---------------------------------------------------------------------
			// temp_xpair    = std::make_pair(cloud->points[i].x,i);// pair_x
			temp_zpair    = std::make_pair(cloud->points[i].z,i);// pair_y
			//      std::cout<<"pair data formate: "<< temp_xpair.first<< std::endl;
			//---------------------------------------------------------------------

			x_list_pair.push_back(temp_xpair);
			z_list_pair.push_back(temp_zpair);
		}

 // choose the left most point, and choose the nearest point
    // choose the left most point, and choose the nearest point
	    int index_zmin   = min_index(z_list_pair);
		int index_xmin   = min_index(x_list_pair);
		std::cout<< "minimum x element index is" << index_xmin<<std::endl;
		std::cout<<  "minimum z element index is" << index_zmin<<std::endl;

		// calculate the radians
		for (int i=0; i < cloud->points.size();i++)
		{
  			if (i != index_zmin)
 		    {
           		double diff_x = cloud->points[i].x - x_list_pair[index_zmin].first;
				double diff_z = cloud->points[i].z - z_list_pair[index_zmin].first;
				//      std::cout<<"difference of z:  "<< diff_z<<"difference of x: "<<diff_x<<std::endl;
				double rad    = atan2(diff_z,diff_x);
				std::pair<double,int> temp_radian = std::make_pair(rad,i);//make radian pair
				radians_all.push_back(temp_radian);
			}
		}

		std::pair<double,int> last_point = std::make_pair(100,index_zmin);
		radians_all.push_back(last_point);//100 representing the start point
		std::sort(radians_all.begin(),radians_all.end(),sortfunction);//sorting the radians
		// put radians into stack
		std::stack<std::pair<double,int>,std::vector<std::pair<double,int> > > ra_stack ( radians_all);
		// radians indices
		std::stack<int,std::vector<int> > ind_stack;
		p_stack.push(cloud->points[radians_all[n-1].second]);// first point 
		ind_stack.push(radians_all[n-1].second);
		p_stack.push(cloud->points[radians_all[0].second]);// second point 
		ind_stack.push(radians_all[0].second);

		for (int i=1; i< n; ++i)
		{
			while(To_left(cloud->points[radians_all[i].second],p_stack)<0)
			{
	    		p_stack.pop();
     		    ind_stack.pop();
   		    }
      		p_stack.push(cloud->points[radians_all[i].second]);
			ind_stack.push(radians_all[i].second);

		}

		int nn = p_stack.size();
  //  pcl::PointCloud<PointT>::Ptr convex_h (new pcl::PointCloud<PointT>);
		for (int i=0;i<nn;++i)
  		{
        	output_points->push_back (p_stack.top());
			std::cout<< p_stack.top()<<std::endl;
			p_stack.pop();

		}
    // print the result
		std::cout<<"the number of convex points is : "<< output_points->size()<<std::endl;
//    pcl::PCDWriter writer;
//    writer.write("convex_hull_1.pcd",*output_points,false);

  };


};

//--------------------------------------------------------------------

//-----------------------Sample Down Algorithm------------------------
class SampleDown_con

{
	public:

			typedef pcl::PointXYZ PointT;

  //container of the downsized sample: hull
			pcl::PointCloud<PointT>::Ptr Sample_hull;

  //constructor 

			SampleDown_con () : Sample_hull(new pcl::PointCloud<PointT>){}

			// distance between points
			double Euclidean_D(PointT & a,PointT & b)

			{
					double dx = a.x-b.x;
					double dz = a.z-b.z;
					double dist = sqrt( dx * dx + dz * dz);
					return (dist);
			}

			// tangent value
			double Tan_radians(PointT & a, PointT & b)
			{
					double dx = b.x - a.x;
					double dz = b.z - a.z;
					double tan_ra = dz/dx;
					//tan_ra = abs(tan_ra);
					return (tan_ra);
			}
			//  Method Definition
			void SampleDown(pcl::PointCloud<PointT>::Ptr &cloud_hull,pcl::PointCloud<PointT>::Ptr & Sample_hull)

			{
                    Sample_hull->clear();
					double tan_cum  = 0;
					double dist_cum = 0;
					Sample_hull->points.push_back (cloud_hull->points[0]);
					for (int i = 0; i< cloud_hull->points.size()-2; ++i)
					{
							double d_tan_1 = Tan_radians (cloud_hull->points[i],cloud_hull->points[i+1]);
							double d_tan_2 = Tan_radians (cloud_hull->points[i+1],cloud_hull->points[i+2]);
							double d_tan_d = d_tan_2 - d_tan_1;

							tan_cum = tan_cum + d_tan_d;
							double dist_1   = Euclidean_D(cloud_hull->points[i],cloud_hull->points[i+1]);
							dist_cum = dist_cum + dist_1;

							if ( dist_cum>0.01 && abs(tan_cum) > 0.9)
							{
									//     Sample_hull->points.push_back (cloud_hull->point[i];
									Sample_hull->points.push_back (cloud_hull->points[i+1]);
									tan_cum  = 0;
									dist_cum = 0;
							}
					}
					// removeDuplivates (Sample_hull);
					// std::cout<<cloud_hull->points[1].x<<std::endl;
					int num = (int)  cloud_hull->points.size();
					Sample_hull->points.push_back(cloud_hull->points[num-1]);
					Sample_hull->width = Sample_hull->points.size();
					Sample_hull->height = 1;
					Sample_hull->is_dense = true;
					std::cout << "Sample_hull has :" << Sample_hull->points.size() << "data points."<<std::endl;

			}
};






#endif
