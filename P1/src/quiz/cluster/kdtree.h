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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// L3-C6::
	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		// L3-C6:: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth % 2; // even & odd
			if (point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}
	// EOTD L3-C6


	// L3-C7:: Search in KdTree
	// return a list of point ids in the tree that are within distance of target
	void searchHelper(Node* node, int depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// Inside the box // why cannot use () in single comparison?
			if ( (node->point[0]>=target[0]-distanceTol && node->point[0]<=target[0]+distanceTol) && (node->point[1]>=target[1]-distanceTol && node->point[1]<=target[1]+distanceTol))
			{
				// Calculate distance
				float dist = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));

				if (dist <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Outside the box
			uint cd = depth % 2;
			if (node->point[cd] > (target[cd] - distanceTol)) // Target: L/U || Current: R/B [if target is left/up to current, abandon right/bottom of current]
			{	// Search next left child 
				searchHelper(node->left, depth+1, target, distanceTol, ids);
			}
			if (node->point[cd] < (target[cd] + distanceTol)) // Target: R/B || Current: L/U
			{
				searchHelper(node->right, depth+1, target, distanceTol, ids);
			}
		}
	}
	
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, 0, target, distanceTol, ids);

		return ids;
	}
};




