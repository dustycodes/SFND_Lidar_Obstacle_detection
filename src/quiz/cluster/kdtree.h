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

	void insertRec(const std::vector<float>& point, const int& id, const uint& currentDepth, Node** currentNode)
    {
        if (*currentNode == NULL)
        {
            *currentNode = new Node(point, id);
            return;
        }

        uint side = currentDepth % 2;
	    if (point[side] < (*currentNode)->point[side])
        {
	        insertRec(point, id, currentDepth + 1, &(*currentNode)->left);
        }
	    else
        {
            insertRec(point, id, currentDepth + 1, &(*currentNode)->right);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        insertRec(point, id, 0, &root);
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		

		return ids;
	}
	

};




