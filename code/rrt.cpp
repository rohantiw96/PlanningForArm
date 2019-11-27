#include "rrt.h"
#define LINKLENGTH_CELLS 10
RRT::RRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int num_samples)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = epsilon;
        num_samples_ = num_samples;
}

std::vector<double> RRT::findNearestNeighbor(const std::vector<double> &q_rand){
    std::vector<double> nearest_neighbor;
    double min_distance = INT_MAX;
    double euclidean_distance = 0;
    for(const auto& n:tree_){
        euclidean_distance = euclideanDistance(n.first,q_rand);
        if(euclidean_distance < min_distance){
            min_distance = euclidean_distance;
            nearest_neighbor = n.first;
        }
    }
    return nearest_neighbor;
}
std::vector<double> RRT::extend(const std::vector<double> &q_start,const std::vector<double> &q_end){
    std::vector<double> q_epsilon;
    for(int i = 0;i <numofDOFs_;i++){
        q_epsilon.push_back(q_end[i] - q_start[i]);
    }
    double norm = getNorm(q_epsilon);
    if (norm <= epsilon_) return q_end;
    for(int i=0;i<numofDOFs_;i++){
        q_epsilon[i] = q_start[i] + q_epsilon[i]/norm * epsilon_;
    }
    return q_epsilon;
}

void RRT::addNode(const std::vector<double> &q_near,const std::vector<double> &q_new){
    tree_[q_new] = q_near;
}

std::vector<double> RRT::interpolate(const std::vector<double> start,const std::vector<double> end){
    std::vector<double> delta;
    std::vector<double> collision_free_configeration;
    std::vector<double> q_current = start;
    for(int i=0;i<numofDOFs_;i++){
        delta.push_back((end[i] - start[i])/num_samples_);
    }
    for(int i=0; i < num_samples_; i++){
        collision_free_configeration = q_current;
        for(int j=0;j<numofDOFs_;j++){
            q_current[j] = q_current[j] + delta[j];
        }
        wrapAngles(q_current);
        if (!IsValidArmConfiguration(q_current)){
            return collision_free_configeration;
        }
    }
    return end;
}

bool RRT::inGoalRegion(const std::vector<double> &angles){
    std::vector<double> diff_vector;
    for(int i=0;i<numofDOFs_;i++){
        diff_vector.push_back(angles[i] - arm_goal_[i]);
    }
    if (getNorm(diff_vector) <= epsilon_){
        return true;
    }
    return false;
}

std::vector<std::vector<double> > RRT::getPath(const std::vector<double> &angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = angles;
    while(tree_[q_current] != arm_start_){
        path.push_back(q_current);
        q_current = tree_[q_current];
    }
    path.push_back(arm_start_);
    std::reverse(std::begin(path), std::end(path));
    return path;
}

double RRT::returnPathCost(){
    return total_cost_;
}

int RRT::returnNumberOfVertices(){
    return tree_.size();
}

void RRT::plan(double*** plan,int* planlength){
    bool reachedGoal = false;
    tree_[arm_start_] = std::vector<double>{};
    std::vector<double> q_new;
    std::vector<double> q_near;
    std::vector<double> q_near_goal;
    std::vector<double> q_epilison;
    std::vector<double> collision_free_configeration;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    int j = 0;
    if (!checkGoalAndStartForCollision()){
        while(!reachedGoal){ 
            q_new = getRandomAngleConfig(0.1,arm_goal_);
            q_near = findNearestNeighbor(q_new);
            q_epilison = extend(q_near,q_new);
            collision_free_configeration = interpolate(q_near,q_epilison);
            if (collision_free_configeration != q_near){
                addNode(q_near,collision_free_configeration);
                if(collision_free_configeration == arm_goal_){
                    printf("Found A Path\n");
                    reachedGoal = true;
                }
            }
            if(j > 50000){
                printf("Coundn't Find A Path\n");
                break;
            }
            j++;
        }
    }
    if(reachedGoal) {
        path = getPath(collision_free_configeration);
        path.push_back(arm_goal_);
        total_cost_ = getPathCost(path);
    }
    else{
        total_cost_ = 0;
    }
    returnPathToMex(path,plan,planlength);
    return;
}