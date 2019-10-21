#pragma once
#include "rrt.h"

class RRTStar: public RRT{
    public:
        RRTStar(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double rewiring_radius);
        void plan(double ***plan,int *planlength);
    private:
        std::unordered_map<std::vector<double>,double, container_hash<std::vector<double>>> cost_;
        std::unordered_map<std::vector<double>,std::vector<std::vector<double>>, container_hash<std::vector<double>>> child_map_;
        double rewiring_radius_;
        
        void addNode(const std::vector<double> &parent,const std::vector<double> &child);
        std::vector<double> getMinCostParent(std::vector<double> q_new,const std::vector<std::vector<double>> &k_nearest_neighbor,double current_cost);
        void rewireNeighboringNode(std::vector<double> q_new,const std::vector<std::vector<double>> &k_nearest_neighbor,double current_cost);
        std::vector<std::vector<double>> findKNearestNeighbor(const std::vector<double> &q_new);
        void updateCostOfChildren(const std::vector<double> &q);
        void updateChildren(const std::vector<double> &parent,const std::vector<double> &child);
        void deleteEdge(std::vector<double> parent,std::vector<double> child);
};