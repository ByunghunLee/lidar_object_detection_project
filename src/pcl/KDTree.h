
#ifndef LIDAR_OBSTACLE_DETECTION_KDTREE_H
#define LIDAR_OBSTACLE_DETECTION_KDTREE_H
template<typename PointT>
class KDTree
{
    class KDNode
    {
        int _id;
        KDNode* _right;
        KDNode* _left;
        std::vector<float> _point;
        KDNode(std::vector<float> point, int id): _point(point), _id(id), _right(nullptr), _left(nullptr){}
    };

private:
    KDNode* root;
};

#endif //LIDAR_OBSTACLE_DETECTION_KDTREE_H
