#ifndef KD_NODE_H
#define KD_NODE_H
#include <iostream>
#include <math.h>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#define KD_TEMPLATE      template <typename X_Point>// use template to solve the 3D and 2D problems
#define KD_NODE          Kd_Node<X_Point> // don't like the template syntax

union TwoD_Point
{
	struct
	{
		float x;
		float y;
	};
	float data[2];
}point;



template<typename X_Point>
class Kd_Node    // node definition

{
	private:
		int _data_di;  // data dimension 
	public:

		int split_axis; // decide which dimention to split
		X_Point node_data; // 2d point
		std::vector<X_Point> Point_List; // point list
		unsigned int id;
//		bool checked; // flag needed for recursive parent check
//		bool orientation;
		int Size_Of(X_Point* &p_0); // check the size of input point
		KD_NODE  (X_Point * p_0, int split);
		KD_NODE* Insert(X_Point* p_0);
		KD_NODE* FindParent(X_Point* p_0);

		static bool equal(X_Point* &a, X_Point* &b, int di);// static equal to global;		
		KD_NODE* Parent;
		KD_NODE* Left;
		KD_NODE* Right;
};



#endif
