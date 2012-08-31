#ifndef KD_TREE_H
#define KD_TREE_h
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include "Kd_Node.h"
#include <pcl/point_types.h>// for using the boost<boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#define KD_TREE     Kd_Tree<X_Point>

KD_TEMPLATE
class Kd_Tree
{
	public:
		/* Member Variables */
		KD_NODE * Root;
		KD_NODE * m_Nearest_N; // nearest neighbour
		std::vector<KD_NODE*> m_K_Nearest_N; // Knearest neighbour 

		/* Member Methods   */

		Kd_Tree(); // constructor 
		void Tree_Build(std::vector<KD_NODE*> & input_points);   // build the tree
		void KNN_search(X_Point* & query_p);  //K nearest Neighbour search 
		static void float Distance2(X_Point* & p_1, X_Point* & p_2, int di); // inline distance function

};








#endif
