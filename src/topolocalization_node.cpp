
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include <memory>
#include <math.h>
#include <vector>
#include <iostream>
#include <string>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


// roscpp
#include "ros/ros.h"
#include "sstream"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"

#include "topomapping/topomapping_node.h"
#include "topomapping/probability_array.h"
#include "topomapping/probability_element.h"