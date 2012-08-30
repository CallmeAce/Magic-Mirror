#include "Kd_tree.h"

Kd_Node::Kd_Node(TwoD_Point* p_0, int split_axis)
{
	twod_point = *p_0;
	axis = split_axis;
}

inline bool Kd_Node::equal(TwoD_Point* &a, TwoD_Point* &b, int di)
{
	for (unsigned int i = 0; i< di; i++)
	{
		if( !(a->data[i]==b.data[i])) // if(a->data[i]!=b.data[i])
			return false;
	}
	return true;
}

// find the parent of a target point, Actually this is confusing, the function should be called "where to put the target point
Kd_Node* Kd_Node::FindParent(TwoD_Point* p_0) 

{
	Kd_Node* parent;
	Kd_Node* next = this;
	int split;
	while (next)// if next node exist;
	{
		split = next->axis;
		parent = next;
		if (p_0->data[split] > next->twod_point.data[split])
			next = next->Right;
		else
			next = next->Left;
	}
	return parent;
}

/* insert the node */

Kd_Node* Kd_Node::Insert(TwoD_Point* p)
{
	Kd_Node* parent = FindParent(p);
	int di = sizeof(p->data)/sizeof(int);
	TwoD_Point * p_1 = parent->twod_point;
	if(equal(p, p_1, di))// equal function prevent duplicate node
		return NULL;
	Kd_Node* newNode = new Kd_Node(p, parent->axis+1 < di? parent->axis+1:0);//rotate the spliting axis

	newNode->Parent = parent;
	if (p->data[parent->axis]> parent->twod_point.data[parent->axis])
	{
		parent->Right =newNode;
		newNode->orientation=1;
	}
	else
	{
		parent->Left = newNode;
		newNode->orientation=0;
	}

}

/* find the nearest neighbour, it start from the root
	find the target parent,then start again from the root and eliminate all halfspaces toofar from the target   */


