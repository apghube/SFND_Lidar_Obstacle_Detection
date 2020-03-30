/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    void insertHelper(Node* &node, uint depth, std::vector<float> point, int id)
	{
		if(node == NULL)
		{
			node=new Node(point,id);
		}
		else
		{
			/*determine the dim to be compared based on the depth, 0=x, 1=y
			if depth is even, dim is x and if depth is odd, dim is y */

			uint cd=depth%2;  

			if(point[cd] < node->point[cd]) 
			{
                insertHelper(node->left,depth+1,point,id);
			}
			else
			{
				insertHelper(node->right,depth+1,point,id);
			}
			
		}
		
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
	    insertHelper(root,0,point,id); //here depth is 0 as root is at 0 level

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




