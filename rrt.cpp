#include "rrt.h"
RRT::RRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int num_samples)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = epsilon;
        num_samples_ = num_samples;
}
void RRT::wrapAngles(std::vector<double> &angles){
    std::vector<double> wrapped_angle;
    for(int i=0;i<angles.size();i++){
        angles[i] = fmod(angles[i],2*PI);
        if (angles[i] < 0)
            angles[i] += 2*PI;
    }
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
    wrapAngles(q_epsilon);
    return q_epsilon;
}
void RRT::addNode(const std::vector<double> &q_near,const std::vector<double> &q_new){
    tree_[q_new] = q_near;
}
std::vector<double> RRT::interpolate(const std::vector<double> &start,const std::vector<double> &end){
    std::vector<double> delta;
    std::vector<double> collision_free_configeration = start;
    for(int i=0;i<numofDOFs_;i++){
        delta.push_back((end[i] - start[i])/ (num_samples_ - 1));
    }
    for(int i=0; i < num_samples_- 1; i++){
        std::vector<double> angles;
        for(int j=0;j<numofDOFs_;j++){
            angles.push_back(start[j]+ delta[j] * i);
        }
        wrapAngles(angles);
        if (!IsValidArmConfiguration(angles)){
            return collision_free_configeration;
        }
        collision_free_configeration = angles;
    }
    return end;
}

bool RRT::inGoalRegion(const std::vector<double> &angles){
    std::vector<double> diff_vector;
    for(int i=0;i<numofDOFs_;i++){
        diff_vector.push_back(angles[i] - arm_goal_[i]);
    }
    if (getNorm(diff_vector)< epsilon_){
        return true;
    }
    return false;
}

std::vector<std::vector<double> > RRT::getPath(const std::vector<double> &angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = angles;
    std::vector<double> q_new;
    // path.push_back(arm_goal_);
    while(tree_[q_current] != arm_start_){
        if(tree_[q_current].empty()) {
            printf("Empty Return\n");
            for(int i=0;i<numofDOFs_;i++){
                printf("This node has no Neighbour %f\n",q_current[i]);
                break;
            }
        }
        path.push_back(q_current);
        q_current = tree_[q_current];
    }
    path.push_back(arm_start_);
    std::reverse(std::begin(path), std::end(path));
    return path;
}

void RRT::returnPathToMex(const std::vector<std::vector<double> >& path,double ***plan,int *planlength){
    *plan = NULL;
    *planlength = path.size();
    if(*planlength > 0){
        *plan = (double**) malloc(*planlength*sizeof(double*));
        for (int i = 0; i < *planlength; i++){
            (*plan)[i] = (double*) malloc(numofDOFs_*sizeof(double)); 
            for(int j = 0; j < numofDOFs_; j++){
                (*plan)[i][j] = path[i][j];
            }
        }
    }
}

void RRT::plan(double*** plan,int* planlength){
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