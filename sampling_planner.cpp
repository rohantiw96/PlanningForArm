#include "sampling_planner.h"

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10
#define PI 3.141592654

SamplingPlanners::SamplingPlanners(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs){
  map_ = map;
  x_size_ = x_size;
  y_size_ = y_size;
  arm_goal_ = arm_goal;
  arm_start_ = arm_start;
  numofDOFs_ = numofDOFs;
  generator_ = std::mt19937(std::random_device()());
  distribution_ = std::uniform_real_distribution<double>(0, 2*PI);
  distribution_goal_selection_ = std::uniform_int_distribution<int>(1, 100);
};

void SamplingPlanners::ContXY2Cell(const double x, const double y, short unsigned int *pX, short unsigned int *pY)
{
  double cellsize = 1.0;
  //take the nearest cell
  *pX = (int)(x / (double)(cellsize));
  if (x < 0)
    *pX = 0;
  if (*pX >= x_size_)
    *pX = x_size_ - 1;

  *pY = (int)(y / (double)(cellsize));
  if (y < 0)
    *pY = 0;
  if (*pY >= y_size_)
    *pY = y_size_ - 1;
}

int SamplingPlanners::getMapIndex(const int x, const int y)
{
  return y * x_size_ + x;
}

void SamplingPlanners::get_bresenham_parameters(const int p1x, const int p1y, const int p2x, const int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
    params->Y1 = p1x;
    params->X1 = p1y;
    params->Y2 = p2x;
    params->X2 = p2y;
  }
  else
  {
    params->X1 = p1x;
    params->Y1 = p1y;
    params->X2 = p2x;
    params->Y2 = p2y;
  }

  if ((p2x - p1x) * (p2y - p1y) < 0)
  {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX = params->X2 - params->X1;
  params->DeltaY = params->Y2 - params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void SamplingPlanners::get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  }
  else
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

int SamplingPlanners::get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

int SamplingPlanners::IsValidLineSegment(const double x0, const double y0, const double x1, const double y1)

{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  //make sure the line segment is inside the environment
  if (x0 < 0 || x0 >= x_size_ ||
      x1 < 0 || x1 >= x_size_ ||
      y0 < 0 || y0 >= y_size_ ||
      y1 < 0 || y1 >= y_size_)
    return 0;

  ContXY2Cell(x0, y0, &nX0, &nY0);
  ContXY2Cell(x1, y1, &nX1, &nY1);

  //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  //iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do
  {
    get_current_point(&params, &nX, &nY);
    if (map_[getMapIndex(nX, nY)] == 1)
      return 0;
  } while (get_next_point(&params));

  return 1;
}

int SamplingPlanners::IsValidArmConfiguration(std::vector<double> angles)
{
  double x0, y0, x1, y1;
  int i;

  //iterate through all the links starting with the base
  x1 = ((double)x_size_) / 2.0;
  y1 = 0;
  for (i = 0; i < numofDOFs_; i++)
  {
    //compute the corresponding line segment
    x0 = x1;
    y0 = y1;
    x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
    y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

    //check the validity of the corresponding line segment
    if (!IsValidLineSegment(x0, y0, x1, y1))
      return 0;
  }
  return 1;
}
std::vector<double> SamplingPlanners::getRandomAngleConfig(const double goal_bias_probability,const std::vector<double> arm_goal){
  std::vector<double> angles;
  if (distribution_goal_selection_(generator_) > goal_bias_probability*100){
    for(int i=0;i<numofDOFs_;i++){
      angles.push_back(distribution_(generator_));
    }
  }
  else{
    angles = arm_goal;
  }
  return angles;
}
