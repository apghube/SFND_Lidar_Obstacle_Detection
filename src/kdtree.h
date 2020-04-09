/* \author Aaron Brown */

//#include "render/render.h"


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

    void searchHelper(Node* node, std::vector<float> target,uint depth,float distanceTol,std::vector<int> &ids)
	{
		uint dim=depth%2; 
		if(node == NULL)
		{
			return;
		}
		else
		{ //target[0]=x, target[1]=y
			if( (((target[0]-distanceTol) <= node->point[0]) && ((target[0]+distanceTol) >= node->point[0])) && (((target[1]-distanceTol) <= node->point[1]) && ((target[1]+distanceTol) >= node->point[1]))) 
		   {
                float distance=sqrt(((node->point[0]- target[0]) * (node->point[0]- target[0])) + ((node->point[1]- target[1]) * (node->point[1]- target[1])));
				if(distance <= distanceTol)
				{
                    ids.push_back(node->id);
				}    
		   }

			if((target[dim]-distanceTol) < node->point[dim]) 
			{
				searchHelper(node->left,target,depth+1,distanceTol,ids);
			}
			if((target[dim]+distanceTol) > node->point[dim])
			{
				searchHelper(node->right,target,depth+1,distanceTol,ids);
			}
		}
		
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root,target,0,distanceTol,ids);
		return ids;
	}
	

};




