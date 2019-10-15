#include "rrt.h"
#include <flann/flann.hpp>
class RRTStar: public RRT{
    public:
        RRTStar(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double rewiring_radius);
        // void plan(double ***plan,int *planlength);
    private:
        std::unordered_map<std::vector<double>,double, container_hash<std::vector<double>>> cost_;
        std::unordered_map<std::vector<double>,std::vector<double>, container_hash<std::vector<double>>> parent_child_;
        double rewiring_radius_;
        void addNode(std::vector<double> parent,std::vector<double> child);
        void rewireNode(std::vector<double> q_new);
        std::vector<double> findNearestNeighbor(std::vector<double> q_rand);


};