function[armplan, armplanlength,time,cost,vertices] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
[armplan, armplanlength,time,cost,vertices] = planner(envmap, armstart, armgoal, planner_id);

