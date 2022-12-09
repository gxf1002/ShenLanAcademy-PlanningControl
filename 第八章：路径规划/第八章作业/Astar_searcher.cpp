#include "carla_shenlan_a_star_planner/Astar_searcher.h"

using namespace std;
using namespace Eigen;

AstarPathFinder::AstarPathFinder() : Node("astart_searcher"){}
AstarPathFinder::~AstarPathFinder(){}

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr **[GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i, j, k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
        return;

    int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for (int i = 0; i < GLX_SIZE; i++)
        for (int j = 0; j < GLY_SIZE; j++)
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if (GridNodeMap[i][j][k]->id == -1) // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    // ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
    Vector3i idx;
    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
        min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
        min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y, const int &idx_z) const
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE &&
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> &neighborPtrSets, vector<double> &edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
 
    // if(currentPtr == nullptr)
    // {
    //     // ROS_ERROR("ERROR!!!");
        
    // }
    Eigen::Vector3i this_node = currentPtr->index;
    int current_x = this_node[0];
    int current_y = this_node[1];
    int current_z = this_node[2];
    auto current_coordinate = currentPtr->coord;
    int neighbour_x;
    int neighbour_y;
    int neighbour_z;
    double distance;
    GridNodePtr temporary_ptr = nullptr;
    Eigen::Vector3d neighbour_coordinate;

    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            for (int k = -1; k <= 1; k++)
            {
                if (i == 0 && j == 0 && k == 0) // current node
                {
                    continue;
                }
                neighbour_x = current_x + i;
                neighbour_y = current_y + j;
                neighbour_z = current_z + k;
                if (neighbour_x < 0 || neighbour_x > (GLX_SIZE - 1) || neighbour_y < 0 || neighbour_y > (GLY_SIZE - 1)|| neighbour_z < 0 || neighbour_z > (GLZ_SIZE - 1))
                // if (isFree(neighbour_x, neighbour_y, neighbour_z))
                {
                    continue; // prevent cross border
                }
                if(isOccupied(neighbour_x, neighbour_y, neighbour_z))
                {
                    continue;
                } 
                
                temporary_ptr = GridNodeMap[neighbour_x][neighbour_y][neighbour_z]; // GrdiNodeMap was initlized in init, represent the map.
                if(temporary_ptr->id == -1)
                {
                    continue;
                }
               
                neighbour_coordinate = temporary_ptr->coord;

                distance = std::sqrt(std::pow((neighbour_coordinate[0] - current_coordinate[0]), 2) + std::pow((neighbour_coordinate[1] - current_coordinate[1]), 2) + std::pow((neighbour_coordinate[2] - current_coordinate[2]), 2));

                neighborPtrSets.push_back(temporary_ptr);
                edgeCostSets.push_back(distance);
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    */
    bool tie_breaker = true;
    double distance_heuristic;
    Eigen::Vector3d node1_coordinate = node1->coord;
    Eigen::Vector3d node2_coordinate = node2->coord;
    // **** TODO: Manhattan *****
    double dx =abs(node1_coordinate(0)-node2_coordinate(0));
    double dy =abs(node1_coordinate(1)-node2_coordinate(1));
    double dz =abs(node1_coordinate(2)-node2_coordinate(2));


    distance_heuristic = (dx + dy + dz);



    // **** TODO: Euclidean  *****
   distance_heuristic =  sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));




    // **** TODO: Diagonal  *****
    double dmin = min({dx, dy, dz});
    double dmax = max({dx, dy, dz});
    double dmid = dx + dy + dz - dmin - dmax;
    distance_heuristic = (sqrt(3)-sqrt(2))*dmin + (sqrt(2)-1)*dmid + dmax;




    if (tie_breaker)
    {
        distance_heuristic = distance_heuristic * (1.0 + 1.0/100.0);
    }

    return distance_heuristic;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    // ros::Time time_1 = ros::Time::now();

    rclcpp::Time time_1 = this->now();
    // ->now();
    // rclcpp::Time end_mpc;
    // start_mpc = this->now();

    // index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    // Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    // openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    // put start node in open set
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->id = 1;
    startPtr->coord = start_pt;
    openSet.insert(make_pair(startPtr->fScore, startPtr));

    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->id = 1;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while (!openSet.empty())
    {
        /* Remove the node with lowest cost function from open set to closed set */
        currentPtr = openSet.begin() -> second;
        openSet.erase(openSet.begin());
        Eigen::Vector3i current_index = currentPtr->index;
        GridNodeMap[current_index[0]][current_index[1]][current_index[2]]->id = -1;

        // if the current node is the goal
        if (currentPtr->index == goalIdx)
        {
            // ros::Time time_2 = ros::Time::now();

            rclcpp::Time time_2 = this->now();
            terminatePtr = currentPtr;
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            std::cout << "[A*]{sucess}  Time in A*  is " << (time_2 - time_1).nanoseconds() / 1000000.0 << "ms, path cost is " << currentPtr->gScore * resolution << "m."  << std::endl;;
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
            
            return;
        }
        // get the succession
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        /*
        For all unexpanded neigbors "m" of node "n"
        */
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            /*
            Judge if the neigbors have been expanded
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set  
            */
           neighborPtr = neighborPtrSets[i];
            if (neighborPtr->id == 0) // discover a new node, which is not in the closed set and open set
            { 
                /*
                As for a new node, put neighbor in open set and record it    
                */
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id=1;
                continue;
            }
            else if (neighborPtr->id == 1) // this node is in open set and need to judge if it needs to update
            { 
                /*
                As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it  
                */
                if(neighborPtr->gScore > (currentPtr->gScore + edgeCostSets[i]))
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }
            else // this node is in closed set
            { 

                continue;
            }
        }
    }
    // if search fails
    // ros::Time time_2 = ros::Time::now();
    // if ((time_2 - time_1).toSec() > 0.1)
    //     ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath()
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    */
    auto middlePtr = terminatePtr;
    while(middlePtr->cameFrom != NULL)
    {
        gridPath.push_back(middlePtr);
        middlePtr = middlePtr->cameFrom;
    }
    for (auto ptr : gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(), path.end());

    return path;
}