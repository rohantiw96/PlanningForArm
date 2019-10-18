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

void PRM::addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor){
    for(auto& c:comopnents_){
        if(c.find(q_neighbor) != c.end()){
            c[q_neighbor].push_back(q_new);
            c[q_new].push_back(q_neighbor);
            break;
        }
    }
}
void PRM::mergeComponents(component_map& m1,component_map& m2){
    if(m1.size() > m2.size()) {
        m1.insert(m2.begin(),m2.end());
        comopnents_.remove(m2);
    }
    else {
        m2.insert(m1.begin(),m1.end());
        comopnents_.remove(m1);
    }
        // for(auto& c:m2){
        //     m1.insert(c);
        // }
}

void PRM::buildRoadMap(){
    int iter = 0;
    std::vector<double> q_rand;
    std::vector<std::vector<double>> k_nearest_neighbors;
    while(iter < num_iteration_){
        q_rand = getRandomAngleConfig(0,std::vector<double>{});
        if (IsValidArmConfiguration(q_rand)){
            k_nearest_neighbors = findKNearestNeighbor(q_rand);
            if(k_nearest_neighbors.size()>0){
                for(auto& neighbors:k_nearest_neighbors){
                    addSample(q_rand,neighbors);
                }
            }
            else{
                comopnents_.emplace_back(component_map(q_rand,std::vector<std::vector<double>>{}));
            }
        }

        iter++;
    }
}
