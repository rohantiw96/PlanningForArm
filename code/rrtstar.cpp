#include "rrtstar.h"

RRTStar::RRTStar(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double rewiring_radius)
        :RRT(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,sampling_rate){
            rewiring_radius_ = rewiring_radius;
        }

void RRTStar::addNode(const std::vector<double>& parent,const std::vector<double>& child){
    tree_[child] = parent;
    cost_[child] = cost_[parent] + euclideanDistance(child,parent);
    child_map_[parent].push_back(child);
}

void RRTStar::deleteEdge(std::vector<double> parent,std::vector<double> child){
    for (int i=0;i<child_map_[parent].size();i++){
        if (child_map_[parent][i] == child){
            child_map_[parent].erase(child_map_[parent].begin()+i);
        }
    }
}

void RRTStar::rewireNeighboringNode(std::vector<double> q_new,const std::vector<std::vector<double>> &k_nearest_neighbor,double current_cost){
    std::vector<double> collision_free_configeration = q_new;
    for(const auto& n:k_nearest_neighbor){
        if(cost_[n] > current_cost + euclideanDistance(n,q_new)){
            collision_free_configeration = interpolate(q_new,n);
            if(collision_free_configeration == n){
                deleteEdge(tree_[n],n);
                addNode(q_new,n);
                updateCostOfChildren(n);
            }
        }
    }
}


std::vector<double> RRTStar::getMinCostParent(std::vector<double> q_new,const std::vector<std::vector<double>> &k_nearest_neighbor,double current_cost){
    std::vector<double> collision_free_configeration;
    std::vector<double> q_min_parent;
    for(const auto& n:k_nearest_neighbor){
        if((cost_[n]+ euclideanDistance(n,q_new)) < current_cost){
            collision_free_configeration = interpolate(n,q_new);
            if(collision_free_configeration == q_new){
                q_min_parent = n;
            }
        }
    }
    return q_min_parent;
}
std::vector<std::vector<double>> RRTStar::findKNearestNeighbor(const std::vector<double> &q_new){
    std::vector<std::vector<double>> k_nearest_neighbor;
    double euclidean_distance = 0;
    for(const auto& n:tree_){
        euclidean_distance = euclideanDistance(n.first,q_new);
        if(euclidean_distance < rewiring_radius_){
            k_nearest_neighbor.push_back(n.first);
        }
    }
    return k_nearest_neighbor;
}

void RRTStar::updateCostOfChildren(const std::vector<double> &q){
    std::queue<std::vector<double>> que;
    que.push(q);
    std::vector<double> current;
    while(!que.empty()){
        current = que.front();
        que.pop();
        for (auto i = child_map_[current].begin(); i != child_map_[current].end(); i++) {
            cost_[*i] =  cost_[current] + euclideanDistance(*i,current);
            if (!child_map_[*i].empty()) {
                que.push(*i);  
            } 
        }
    }
}

void RRTStar::updateChildren(const std::vector<double> &parent,const std::vector<double> &child){
    for(int i=0;i<child_map_[parent].size();i++){
        if (child_map_[parent][i] == child){
            child_map_[parent].empty();
        }
    }
}

void RRTStar::plan(double ***plan,int *planlength){
    bool reachedGoal = false;
    tree_[arm_start_] = arm_start_;
    cost_[arm_start_] = 0.0;
    int j = 0;
    std::vector<double> q_new;
    std::vector<double> q_near;
    std::vector<double> q_epilison;
    std::vector<double> q_min_parent;
    std::vector<double> collision_free_configeration;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    std::vector<std::vector<double>> kNearestNeighbors;
    if (!checkGoalAndStartForCollision()){
        while(!reachedGoal){
            q_new = getRandomAngleConfig(0.1,arm_goal_);
            q_near = findNearestNeighbor(q_new);
            q_epilison = extend(q_near,q_new);
            collision_free_configeration = interpolate(q_near,q_epilison);

            if (collision_free_configeration != q_near){
                addNode(q_near,collision_free_configeration);
                kNearestNeighbors = findKNearestNeighbor(collision_free_configeration);
                if(!kNearestNeighbors.empty()){
                    q_min_parent = getMinCostParent(collision_free_configeration,kNearestNeighbors,cost_[collision_free_configeration]);
                    if(!q_min_parent.empty()) {
                        deleteEdge(tree_[collision_free_configeration],collision_free_configeration);
                        addNode(q_min_parent,collision_free_configeration);
                    }
                }
                rewireNeighboringNode(collision_free_configeration,kNearestNeighbors,cost_[collision_free_configeration]);
                if(collision_free_configeration == arm_goal_){
                    reachedGoal = true;
                }
            }
            if(j>50000){
                printf("Coundn't Find A Path\n");
                break;
            }
            j++;
        }
    }
    if(reachedGoal) {
        path = getPath(collision_free_configeration);
        total_cost_ = getPathCost(path);
    }
    else{
        total_cost_ = 0;
    }
    returnPathToMex(path,plan,planlength);
    return;
}


