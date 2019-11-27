#include "prm.h"

PRM::PRM(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int num_samples,int num_iteration)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = epsilon;
        num_iteration_ = num_iteration;
        num_samples_ = num_samples;
}
bool PRM::interpolate(const std::vector<double> &start,const std::vector<double> &end){
    std::vector<double> delta;
    for(int i=0;i<numofDOFs_;i++){
        delta.push_back((end[i] - start[i])/ (num_samples_ - 1));
    }
    for(int i=0; i < num_samples_- 1; i++){
        std::vector<double> angles;
        for(int j=0;j<numofDOFs_;j++){
            angles.push_back(start[j]+ delta[j] * i);
        }
        if (!IsValidArmConfiguration(angles)){
            return false;
        }
    }
    return true;
}

std::vector<std::vector<double>> PRM::findKNearestNeighbor(const std::vector<double> &q_new){
    std::vector<std::vector<double>> k_nearest_neighbor;
    double euclidean_distance = 0;
    for(const auto& c:comopnents_){
        for(const auto& m:c){
            euclidean_distance = euclideanDistance(m.first,q_new);
            if(euclidean_distance < epsilon_ && interpolate(q_new,m.first)){
                k_nearest_neighbor.push_back(m.first);
            }
        }
    }
    return k_nearest_neighbor;
}

std::vector<double> PRM::findNearestNeighbor(const std::vector<double> &q_new){
    std::vector<double> nearest_neighbor;
    double euclidean_distance = 0;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& c:comopnents_){
        for(const auto& m:c){
            euclidean_distance = euclideanDistance(m.first,q_new);
            if(euclidean_distance < min_distance && interpolate(q_new,m.first)){
                min_distance = euclidean_distance;
                nearest_neighbor = m.first;
            }
        }
    }
    return nearest_neighbor;
}

void PRM::addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor){
    for(auto& c:comopnents_){
        if(c.find(q_neighbor) != c.end()){
            c[q_neighbor].push_back(q_new);
            c[q_new].push_back(q_neighbor);
            break;
        }
    }
}
int PRM::findComponent(std::vector<double> &q_neighbor){
    int num_component = 0;
    for(auto& c:comopnents_){
        if(c.find(q_neighbor) != c.end()){
            break;
        }
        num_component++;
    }
    return num_component;
}
void PRM::mergeComponents(component_map &m1,component_map &m2){
    if(m1.size() > m2.size()) {
        m1.insert(m2.begin(),m2.end());
        comopnents_.remove(m2);
    }
    else {
        m2.insert(m1.begin(),m1.end());
        comopnents_.remove(m1);
    }
}
void PRM::addStartAndGoalNode(){
    std::vector<double> start_neighbor = findNearestNeighbor(arm_start_);
    std::vector<double> goal_neighbor = findNearestNeighbor(arm_goal_);
    addSample(arm_start_,start_neighbor);
    addSample(arm_goal_,goal_neighbor);
}

std::vector<std::vector<double>> PRM::backTrack(std::vector<double> node){
    std::vector<double> current_angle = node;
    std::vector<std::vector<double>> path;
    while (current_angle != arm_start_) // Backtracking to get the shortest path
    {
        current_angle = came_from_[current_angle];
        path.emplace_back(current_angle);
    }
    path.push_back(arm_start_);
    std::reverse(path.begin(),path.end());
    return path;
}

int PRM::returnNumberOfVertices(){
    return (*std::next(comopnents_.begin(), findComponent(arm_goal_))).size();
}

std::vector<std::vector<double>> PRM::getShortestPath(){
    int goal_component = findComponent(arm_goal_);
    if (goal_component != findComponent(arm_start_)){
        printf("Start and Goal don't belong to the same components\n");
        return std::vector<std::vector<double>>{};
    }
    else{
        std::unordered_map<std::vector<double>,double,container_hash<std::vector<double>>> dijkstra_cost_;
        component_map m = *std::next(comopnents_.begin(), goal_component);
        for(const auto& nodes:m){
            dijkstra_cost_[nodes.first] = std::numeric_limits<double>::max();
        }
        std::priority_queue<std::vector<double>> list;
        dijkstra_cost_[arm_start_] = 0.0; // Cost of tbe initial node 0
        list.push(arm_start_);
        double cost = 0;
        std::vector<std::vector<double>> neighbors;
        std::vector<double> current;
        while(!list.empty()){
            current = list.top();
            list.pop();
            neighbors = m.find(current)->second;
            for(const auto& n:neighbors){
                cost = euclideanDistance(current,n) + dijkstra_cost_[current];
                if (cost < dijkstra_cost_[n]){
                    dijkstra_cost_[n] = cost;
                    list.push(n);
                    came_from_[n] = current;
                }
            }
        }
    }
    return backTrack(arm_goal_);
}

void PRM::buildRoadMap(){
    int iter = 0;
    std::vector<double> q_rand;
    std::vector<std::vector<double>> k_nearest_neighbors;
    int own_component = -1;
    int neighbor_component = -1;
    while(iter < num_iteration_){
        q_rand = getRandomAngleConfig(0,std::vector<double>{});
        if (IsValidArmConfiguration(q_rand)){
            k_nearest_neighbors = findKNearestNeighbor(q_rand);
            if(k_nearest_neighbors.size()>0){
                addSample(q_rand,*k_nearest_neighbors.begin()); // Add the sample to the componenent of the first neighbor
                own_component = findComponent(q_rand);
                for (auto neighbors = std::next(k_nearest_neighbors.begin()); neighbors != k_nearest_neighbors.end(); ++neighbors){
	                neighbor_component = findComponent(*neighbors);
                    if(own_component!=neighbor_component){
                        mergeComponents(*std::next(comopnents_.begin(), own_component),*std::next(comopnents_.begin(), neighbor_component));
                        addSample(q_rand,*neighbors);
                        own_component = findComponent(q_rand);
                    }
                }   
            }
            else{
                std::unordered_map<std::vector<double>, std::vector<std::vector<double>>, container_hash<std::vector<double>>> new_component;
                new_component[q_rand] = std::vector<std::vector<double>>{};
                comopnents_.emplace_back(new_component);
            }
        }
        iter++;
    }
}
double PRM::returnPathCost(){
    return total_cost_;
}
void PRM::plan(double ***plan,int *planlength){
    total_cost_= 0;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    if (!checkGoalAndStartForCollision()){
        buildRoadMap();
        addStartAndGoalNode();
        path =  getShortestPath();
        if (path.size() > 0) total_cost_ = getPathCost(path);
    }
    returnPathToMex(path,plan,planlength);
}
