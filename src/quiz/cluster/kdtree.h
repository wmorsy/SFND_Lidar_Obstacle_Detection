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

    void insertHelper(Node *&node, uint depth, std::vector<float> point, int id) {

        // Empty node
        if(node == NULL)
            node = new Node(point,id);
        else {
            // Calculate current dimension
            uint dim = depth % 2;

            if(point[dim] < (node)->point[dim]) {
                insertHelper(node->left, depth+1, point, id);
            }
            else {
                insertHelper(node->right, depth+1, point, id);
            }
        }
    }

	void insert(std::vector<float> point, int id)
	{
		// DONE: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root, 0, point, id);
	}

    void searchHelper(const std::vector<float> target, const Node* node, const int depth,
                        const float distanceTol, std::vector<int>& ids) const {
        if (node != NULL) {
          const float left_boundary = target[0] - distanceTol;
          const float right_boundary = target[0] + distanceTol;
          const float lower_boundary = target[1] - distanceTol;
          const float upper_boundary = target[1] + distanceTol;

          if (((node->point[0] >= left_boundary) &&
               (node->point[0] <= right_boundary)) &&
              ((node->point[1] >= lower_boundary) &&
               (node->point[1] <= upper_boundary))) {
            float dist_x = node->point[0] - target[0];
            float dist_y = node->point[1] - target[1];
            float distance = sqrt(dist_x * dist_x + dist_y * dist_y);
            if (distance <= distanceTol) 
                ids.push_back(node->id);
          }

          if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
            searchHelper(target, node->left, depth + 1, distanceTol, ids);
          if ((target[depth % 2] + distanceTol > node->point[depth % 2]))
            searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(const std::vector<float> target, const float distanceTol) const
    {
        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);

        return ids;
    }
};




