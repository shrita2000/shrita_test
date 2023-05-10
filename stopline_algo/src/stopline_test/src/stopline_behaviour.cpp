#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>

#include <aux_lib/struct_defs.h>
#include <aux_lib/aux_functions.h>
#include <aux_lib/qp_problem.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <traffic_msgs/CenterLanes.h>
#include <traffic_msgs/Decision.h>
#include <traffic_msgs/DecisionTrajectory.h>

#include <eigen3/Eigen/Dense>

#include "matplotlibcpp.h"