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

    void searchRec(std::vector<int>& ids, const Node* node, const uint& currentDepth, const std::vector<float>& target, const float& distanceTol)
    {
        if (node == nullptr)
        {
            return;
        }

        // Check first coordinate
        if ((node->point[0] >= target[0] - distanceTol) && (node->point[0] <= target[0] + distanceTol))
        {
            // Check second coordinate
            if ((node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol))
            {
                // Actual distance calculation
                auto distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }
        }

        // Continue search
        if (target[currentDepth % 2] - distanceTol < node->point[currentDepth % 2])
        {
            searchRec(ids, node->left, currentDepth + 1, target, distanceTol);
        }
        if (target[currentDepth % 2] + distanceTol > node->point[currentDepth % 2])
        {
            searchRec(ids, node->right, currentDepth + 1, target, distanceTol);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRec(ids, root, 0, target, distanceTol);
		return ids;
	}
	

};




