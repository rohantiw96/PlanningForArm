#include"rrtconnect.h"

RRTConnect::RRTConnect(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate)
    :RRT(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,sampling_rate){
}

void RRTConnect::addNode(const std::vector<double> &q_near,const std::vector<double> &q_new,bool is_goal){
    if (q_near == q_new){
        printf("Same Node Being Added\n");
    }
    if (is_goal) tree_goal_[q_near] = q_new;
    else tree_[q_new] = q_near;
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


void RRTConnect::plan(double*** plan,int* planlength){
    bool reachedGoal = false;
    tree_[arm_start_] = std::vector<double>{};
    tree_goal_[arm_goal_] = arm_goal_;
    std::vector<double> q_new;
    std::vector<double> q_near;
    std::vector<double> q_near_goal;
    std::vector<double> q_epilison;
    std::vector<double> collision_free_configeration;
    std::vector<double> collision_free_configeration_other;
    std::vector<std::vector<double>> path;
    bool is_goal = false;
    int j = 0;
    while(!reachedGoal){
        q_new = getRandomAngleConfig(0,arm_goal_);
        q_near = findNearestNeighbor(q_new,is_goal);
        q_epilison = extend(q_near,q_new);
        collision_free_configeration = interpolate(q_near,q_epilison);
        if (collision_free_configeration != q_near){
            addNode(q_near,collision_free_configeration,is_goal);
            q_near_goal = findNearestNeighbor(collision_free_configeration,!is_goal);
            collision_free_configeration_other = interpolate(collision_free_configeration,q_near_goal);
            if (collision_free_configeration_other != q_near_goal){
                addNode(q_near_goal,collision_free_configeration_other,!is_goal);
                if(collision_free_configeration == collision_free_configeration_other){
                    printf("Start Tree Size : %d\n",tree_.size());
                    printf("End Tree Size: %d\n",tree_goal_.size());
                    // for(int i=0;i<numofDOFs_;i++){
                    //     printf("Last node in the Goal Tree %f\n",collision_free_configeration_other[i]);
                    // }
                    // printf("Size of the goal node Successor: %d\n",tree_goal_[arm_goal_].size());
                    tree_.insert(tree_goal_.begin(), tree_goal_.end());
                    reachedGoal = true;
                }
            }
        }
        // if(j >= 1){
        //     reachedGoal = true;
        // }
        // j++;
    }
    if(reachedGoal) {
        path = getPath(arm_goal_);
    }
    returnPathToMex(path,plan,planlength);
    return;
}


