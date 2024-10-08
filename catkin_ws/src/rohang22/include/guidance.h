#ifndef __GUIDANCE_H__
#define __GUIDANCE_H__

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

#include "global_def.h"
#include "config.h"
#include "algebra.h"

double get_angle(std::vector<double> start, std::vector<double> end);
std::vector<double> line_guidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step);
std::vector<double> circle_guidance(std::vector<double> center, double radius, double direc, std::vector<double> local, double step);


#endif