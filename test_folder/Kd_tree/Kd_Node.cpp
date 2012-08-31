#include "KD_NODE.h"

/* constructor */
KD_TEMPLATE
KD_NODE::Kd_Node(X_Point* &p_0, int split)
{
	node_data = *p_0;
	split_axis = split;
	_data_di = Size_Of(p_0);
}


/*    check the point size */
int KD_NODE::Size_Of(X_Point* & p_0)
{
	sizeof(p->data)/sizeof(float);
}

/* the equal function defined as a static function, so it can be called directly by other class */
KD_TEMPLATE
bool KD_NODE::equal(X_Point* &a, X_Point* &b, int di)
{
	for (unsigned int i = 0; i< di; i++)
	{
		if( !(a->data[i]==b.data[i])) // if(a->data[i]!=b.data[i])
			return false;
	}
	return true;
}


/* find the parent of a target point, Actually this is confusing, the function should be called "where to put the target point */
KD_TEMPLATE
KD_NODE* KD_NODE::FindParent(X_Point* p_0) 
{
	KD_NODE* parent;
	KD_NODE* child = this;
	int split;
	while (child)// if child node exist;
	{
		split = child->split_axis;
		parent = child;
		if (p_0->data[split] > child->node_data.data[split])
			child = child->Right;
		else
			child = child->Left;
	}
	return parent;
}



/* insert the node */


KD_NODE* KD_NODE::Insert(X_Point* p_0)
{
	KD_NODE* parent = FindParent(p_0);
//	int di = sizeof(p->data)/sizeof(int);// dimension of data
	X_Point * p_1 = &parent->node_data;
	if(equal(p_0, p_1, _data_di))// equal function prevent duplicate node
		return NULL;
	KD_NODE* newNode = new Kd_Node(p, parent->split_axis+1 < di? parent->split_axis+1:0);//rotate the spliting split_axis

	newNode->Parent = parent;
	if (p_0->data[parent->split_axis]> parent->node_data.data[parent->split_axis])
	{
		parent->Right =newNode;
//		newNode->orientation=1;
	}
	else
	{
		parent->Left = newNode;
//		newNode->orientation=0;
	}

}

/* find the nearest neighbour, it start from the root
	find the target parent,then start again from the root and eliminate all halfspaces toofar from the target   */


