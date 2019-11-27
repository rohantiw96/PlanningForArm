#include"rrtconnect.h"

RRTConnect::RRTConnect(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate)
    :RRT(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,sampling_rate){
}

void RRTConnect::addNode(const std::vector<double> &q_near,const std::vector<double> &q_new,bool is_goal){
    if (is_goal) {
        tree_goal_[q_new] = q_near;
    }
    else {
        tree_[q_new] = q_near;
    }
}

std::vector<double> RRTConnect::findNearestNeighbor(const std::vector<double> &q_rand,const bool is_goal){
    std::vector<double> nearest_neighbor;
    double min_distance = INT_MAX;
    double euclidean_distance = 0;
    if (is_goal){
        for(const auto& n:tree_goal_){
            euclidean_distance = euclideanDistance(n.second,q_rand);
            if(euclidean_distance < min_distance){
                min_distance = euclidean_distance;
                nearest_neighbor = n.second;
            }
        }
    }
    else{
        for(const auto& n:tree_){
            euclidean_distance = euclideanDistance(n.first,q_rand);
            if(euclidean_distance < min_distance){
                min_distance = euclidean_distance;
                nearest_neighbor = n.first;
            }
        }
    }
    return nearest_neighbor;
}
std::vector<double> RRTConnect::extendNode(std::vector<double> q_new,const bool is_goal){
    std::vector<double> q_near= findNearestNeighbor(q_new,is_goal);
    std::vector<double> q_epilison = extend(q_near,q_new);
    std::vector<double> collision_free_configeration = interpolate(q_near,q_epilison);
    if (collision_free_configeration != q_near){
        addNode(q_near,collision_free_configeration,is_goal);
        return collision_free_configeration;
    }
    return std::vector<double>{};   
}

std::vector<double> RRTConnect::joinNode(std::vector<double> q_exteded,const bool is_goal){
    std::vector<double> q_near= findNearestNeighbor(q_exteded,is_goal);
    std::vector<double> collision_free_configeration = interpolate(q_near,q_exteded);
    if (collision_free_configeration != q_near){
        addNode(q_near,collision_free_configeration,is_goal);
        return collision_free_configeration;
    }
    return std::vector<double>{};
}
std::vector<std::vector<double> > RRTConnect::getPathToGoal(const std::vector<double> &angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = angles;
    while(tree_goal_[q_current] != arm_goal_){
        path.push_back(q_current);
        q_current = tree_goal_[q_current];
    }
    path.push_back(arm_goal_);
    return path;
}

void RRTConnect::plan(double*** plan,int* planlength){
    bool reachedGoal = false;
    tree_[arm_start_] = std::vector<double>{};
    tree_goal_[arm_goal_] = arm_goal_;
    std::vector<double> q_new;
    std::vector<double> collision_free_configeration;
    std::vector<double> collision_free_configeration_other;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    std::vector<std::vector<double>> path_to_goal;
    bool is_goal = true;
    int j = 0;
    if (!checkGoalAndStartForCollision()){
        while(!reachedGoal){
            q_new = getRandomAngleConfig(0,arm_goal_);
            collision_free_configeration = extendNode(q_new,!is_goal);
            if (!collision_free_configeration.empty()){
                collision_free_configeration_other = joinNode(collision_free_configeration,is_goal);
                if (!collision_free_configeration_other.empty()){
                    if(collision_free_configeration == collision_free_configeration_other){
                        reachedGoal = true;
                    }
                }
            }
            is_goal = !is_goal;
            if(j>50000){
                printf("Coundn't Find A Path\n");
                break;
            }
            j++;
        }
    }
    if(reachedGoal) {
        path = getPath(collision_free_configeration);
        path_to_goal = getPathToGoal(collision_free_configeration);
        path.insert(path.end(), path_to_goal.begin(), path_to_goal.end());
        total_cost_ = getPathCost(path);
    }
    else{
        total_cost_ = 0;
    }
    returnPathToMex(path,plan,planlength);
    return;
}


