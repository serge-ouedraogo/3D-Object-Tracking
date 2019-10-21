/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "render/render.h"
//#include "render/box.h"


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
  
  void inserthelper(Node **root, const std::vector<float>& point, int depth, int id)
  {
    if(*(root) == NULL)
    {
      *(root) = new Node(point, id); 
    }
    else
    {
      int cd = depth%2;
      if(point[cd] < ((*root)->point[cd]))
      {
        inserthelper(&((*root))->left, point, depth + 1, id);
      }
      else
      {
        inserthelper(&((*root))->right, point, depth + 1, id);
      }
    }
  }
  
	void insert(const std::vector<float>& point, int id)
	{
      // TODO: Fill in this function to insert a new point into the tree
      // the function should create a new node and place correctly with in the root 
    inserthelper(&root, point, 0, id);
        
	}

  void searchhelper(const std::vector<float>& target, Node *node, int depth, float distTol, std::vector<int>& ids)
  {
    if(node !=NULL)
    {
      if( (node->point[0]) >= (target[0] - distTol) && (node->point[0]) <= (target[0] + distTol) )
      {
       float distance = sqrt(((node->point[0] - target[0]) * (node->point[0] - target[0])) +                             ((node->point[1] - target[1]) * (node->point[1] - target[1])));
        if(distance < distTol)
        {
          ids.push_back(node->id);
        }
      }
      if((target[depth%2] - distTol) < node->point[depth%2] )
      {
        searchhelper(target, node->left, depth+1, distTol, ids);
      }
      
      if((target[depth%2] + distTol) > node->point[depth%2] )
      {
        searchhelper(target, node->right, depth+1, distTol, ids);
      }
      
    }
  }
  // return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol)
	{
      std::vector<int> ids;
      searchhelper(target, root, 0, distanceTol, ids);
      return ids;
	}
	

};

