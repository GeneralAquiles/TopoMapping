#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include <math.h>
#include <vector>

#include "sstream"

/**
 * @brief Given a field of view and number of samples, calculates the angle (refered to 0) for each sample.
 * Extract the measurements resolution from the FOV/N and then, assings the angular possition of each meassure.
 * 
 * @param fov Field of view. Needs to be in degrees.
 * @param N   Number of samples
 * @param angles OUTPUT. Vector storing the calculated angles, with size N.
 */
void calculate_angles(double fov, int N, std::vector<double> &angles);

/**
 * @brief Get the mean of an array of values of the sensor. It discards any measure below 0.001 and above 100
 * 
 * @param dist Arrays of desired measures to calculate the mean.
 * @return The mean. [double]
 */
double get_mean(std::vector<double> &dist);

/**
 * @brief Returns True if the distance in between robot's sides matched any of the defined door sizes
 * 
 * @param angle Array of angles from laser measures
 * @param distance  Array of laser measures
 * @param N     Total of measures
 * @param fov   Field of View
 * @return true     If door found
 * @return false    Any other case
 * 
 */
bool detect_door(std::vector<double> &angle, std::vector<double> &distance, int N, double fov);
void get_laser_data(const sensor_msgs::LaserScan::ConstPtr& msg);