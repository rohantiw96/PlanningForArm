#include "sampling_planner.h"
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <functional>

template <typename Container> 
struct container_hash {
    std::size_t operator()(Container const& c) const {
        return boost::hash_range(c.begin(), c.end());
    }
};
class RRT: public SamplingPlanners{
    public:
        RRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate);
        virtual void plan(double ***plan,int *planlength);
    protected:
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_;
        double epsilon_;
        int num_samples_;
        double euclideanDistance(const std::vector<double> &q_1,const std::vector<double> &q_2);
        virtual std::vector<double> findNearestNeighbor(const std::vector<double> &q_rand);
        std::vector<double> extend(const std::vector<double> &q_start,const std::vector<double> &q_end);
        virtual void addNode(const std::vector<double> &q_near,const std::vector<double> &q_new);
        double getNorm(const std::vector<double> &vec);
        bool inGoalRegion(const std::vector<double> &angles);
        std::vector<double> interpolate(const std::vector<double> &start,const std::vector<double> &end);
        std::vector<std::vector<double> > getPath(const std::vector<double> &start);
        void returnPathToMex(const std::vector<std::vector<double> >& path,double ***plan,int *planlength);
        void wrapAngles(std::vector<double> &angles);
};