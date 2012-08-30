#ifndef KD_TREE_H
#define KD_TREE_H
#include <iostream>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

union TwoD_Point
{
	struct
	{
		int x;
		int y;
	};
	int data[2];
}point;


class Kd_Node

{
	public:

		int axis; // split dimension
		TwoD_Point twod_point; // 2d point
		std::vector<TwoD_Point> Point_List; // point list
		unsigned int id;
		bool checked; // flag needed for recursive parent check
		bool orientation;
	
		Kd_Node(TwoD_Point * p_0, int split_axis);
		Kd_Node* Insert(TwoD_Point* p);
		Kd_Node* FindParent(TwoD_Point* p_0);

		static bool equal(TwoD_Point* &a, TwoD_Point* &b, int di);// static equal to global;		
		Kd_Node* Parent;
		Kd_Node* Left;
		Kd_Node* Right;
};

class Kd_Tree
{
	

}

#endif
