#include <iostream>
#include <math.h>
#include <random>
#include <chrono>
#define PI 3.141592654

typedef struct
{
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
} bresenham_param_t;

class SamplingPlanners
{
public:
    SamplingPlanners(double *map,
                     int x_size,
                     int y_size,
                     const std::vector<double> &arm_start,
                     const std::vector<double> &arm_goal,
                     int numofDOFs);

protected:
    double *map_;
    int x_size_;
    int y_size_;
    std::vector<double> arm_start_;
    std::vector<double> arm_goal_;
    int numofDOFs_;
    double ***plan_;
    int *planlength_;
    std::random_device rand_device_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> distribution_;
    std::uniform_int_distribution<int> distribution_goal_selection_;
    void ContXY2Cell(const double x, const double y, short unsigned int *pX, short unsigned int *pY);
    void get_bresenham_parameters(const int p1x, const int p1y, const int p2x, const int p2y, bresenham_param_t *params);
    void get_current_point(bresenham_param_t *params, int *x, int *y);
    int get_next_point(bresenham_param_t *params);
    int IsValidLineSegment(const double x0, const double y0, const double x1, const double y1);
    int IsValidArmConfiguration(std::vector<double> angles);
    int getMapIndex(const int x, const int y);
    std::vector<double> getRandomAngleConfig(const double goal_bias_probability,const std::vector<double> arm_goal);
    double euclideanDistance(const std::vector<double> &q_1,const std::vector<double> &q_2);
    double getNorm(const std::vector<double>& vec);
};
