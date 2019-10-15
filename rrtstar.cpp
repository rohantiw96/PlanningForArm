#include "rrtstar.h"
flann::Index<flann::L2_Simple<double>> kdtree_ = flann::KDTreeSingleIndexParams();
RRTStar::RRTStar(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double rewiring_radius)
        :RRT(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,sampling_rate){
            kdtree_.buildIndex(flann::Matrix<double>(&arm_start_[0],1,numofDOFs_));
            rewiring_radius_ = rewiring_radius;
        }

void RRTStar::addNode(std::vector<double> parent,std::vector<double> child){
    kdtree_.addPoints(flann::Matrix<double>(&child[0],1,numofDOFs_));
    tree_[child] = parent;
}

std::vector<double> RRTStar::findNearestNeighbor(std::vector<double> q_rand){
    std::vector< std::vector<int> > indices;
    std::vector<std::vector<double> > dists;
    
    kdtree_.knnSearch(flann::Matrix<double>(&q_rand[0],1,numofDOFs_),indices,dists,1,flann::SearchParams());
    // double* q_near = kdtree_.getPoint(indices[0][0]);
    std::vector<double> q_near(kdtree_.getPoint(indices[0][0]),kdtree_.getPoint(indices[0][0]) + numofDOFs_);
    return q_near;
}

void RRTStar::rewireNode(std::vector<double> q_new){
    kdtree_.knnSearch();
}

void RRT::plan(double ***plan,int *planlength){
    bool reachedGoal = false;
    tree_[arm_start_] = arm_start_;
    std::vector<double> q_new;
    std::vector<double> q_near;
    std::vector<double> q_epilison;
    std::vector<double> collision_free_configeration;
    std::vector<std::vector<double>> path;
    while(!reachedGoal){
        q_new = getRandomAngleConfig(0.1,arm_goal_);
        q_near = findNearestNeighbor(q_new);
        q_epilison = extend(q_near,q_new);
        collision_free_configeration = interpolate(q_near,q_epilison);
        if (collision_free_configeration != q_near){
            addNode(q_near,collision_free_configeration);
            if(collision_free_configeration == arm_goal_){
                reachedGoal = true;
            }
        }
    }
    if(reachedGoal) {
        path = getPath(collision_free_configeration);
    }
    returnPathToMex(path,plan,planlength);
    return;
}


