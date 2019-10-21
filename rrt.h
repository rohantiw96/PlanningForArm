#pragma once
#include "sampling_planner.h"

class RRT: protected SamplingPlanners{
    public:
        RRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate);
        void plan(double ***plan,int *planlength);
        double returnPathCost();
        int returnNumberOfVertices();
    protected:
        double epsilon_;
        int num_samples_;
        double total_cost_;
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_;
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_rand);
        std::vector<double> extend(const std::vector<double> &q_start,const std::vector<double> &q_end);
        void addNode(const std::vector<double> &q_near,const std::vector<double> &q_new);
        bool inGoalRegion(const std::vector<double> &angles);
        std::vector<double> interpolate(const std::vector<double> start,const std::vector<double> end);
        std::vector<std::vector<double> > getPath(const std::vector<double> &start);
};