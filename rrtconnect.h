#pragma once
#include "rrt.h"

class RRTConnect: public RRT{
    public:
        RRTConnect(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate);
        void plan(double ***plan,int *planlength);
    private:
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_goal_;
        void addNode(const std::vector<double> &q_near,const std::vector<double> &q_new,bool is_goal);
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_rand,const bool is_goal);
        std::vector<double> extendNode(std::vector<double> q_new,bool is_goal);
        std::vector<double> joinNode(std::vector<double> q_new,bool is_goal);
        std::vector<std::vector<double>> getPathToGoal(const std::vector<double> &angles);
};