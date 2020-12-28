#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "sstream"

#include <math.h>
#include <vector>

const double pi = 3.14159265;

ros::Publisher door_pub_;
//#######################################################################
double inner_angle_range = 6;
double outer_angle_range = 60;
std::vector<double> door_wall_clearance = {0.3, 0.2}; 
std::vector<double>  door_size = {0.9, 1.75};
std::vector<double>  door_offset = {0.1, 0.15};
//#######################################################################


void calculate_angles(double fov, int N, std::vector<double> &angles){

  double res = fov/(N-1);
  double offset = (fov-180)/2.0;

  for (int i = 0; i < N; i++){ 
    angles[i] = (i*res)-offset;
  }
}

double get_mean(std::vector<double> &dist){
  double total = 0.0;
  int correct_measures = 0;

  for (int i=0; i<dist.size(); i++){
    if ( (100.0 > dist[i]) && (dist[i] > 0.001) ){
        total+=dist[i]; 
        correct_measures++; 
    } 
  }

  if ( correct_measures != 0)  return total/correct_measures;
  else return 0;
}

double get_door_width(std::vector<double> &dist, std::vector<double> &angle){
  double X_a = dist[0] * cos(angle[0]);
  double Y_a = dist[0] * sin(angle[0]);
  double X_b = dist[dist.size()-1] * cos(angle[dist.size()-1]);
  double Y_b = dist[dist.size()-1] * sin(angle[dist.size()-1]);

  double width = sqrt( pow(X_a-X_b, 2) + pow(Y_a-Y_b, 2) );
  std::cout << "Door width: " << width << std::endl;
}

bool detect_door(std::vector<double> &angle, std::vector<double> &distance, int N, double fov){

  double right_distance, left_distance, outer_right_distance, outer_left_distance;
  double res=fov/(N-1);
  int door_measures = int(inner_angle_range/res);
  int outer_measures = int(outer_angle_range/res);

  std::vector<double> door_right (door_measures);
  std::vector<double> anlge_right (door_measures);
  std::vector<double> door_left (door_measures);
  std::vector<double> outer_right (outer_measures);
  std::vector<double> outer_left (outer_measures);

 
  for (int i = 0; i < N; i++){

    double D_angle = angle[i];

    if ( ((0+outer_angle_range/2) > D_angle) && (D_angle > (0-outer_angle_range/2)) && (std::isfinite(distance[i])) ){
      outer_right.push_back(distance[i]);
      if ( ((0+inner_angle_range/2) > D_angle) && (D_angle > (0-inner_angle_range/2)) ){
        door_right.push_back(distance[i]);
        anlge_right.push_back(distance[i]);
      }
    }
    else if ( ((180+outer_angle_range/2) > D_angle) && (D_angle > (180-outer_angle_range/2)) && (std::isfinite(distance[i])) ){
      outer_left.push_back(distance[i]);
      if ( ((180+inner_angle_range/2) > D_angle) && (D_angle > (180-inner_angle_range/2)) ){
        door_left.push_back(distance[i]);
      }
    }
  }

  
  if ( (sizeof(door_right) > 0) && (sizeof(door_left) > 0) ){
    right_distance = get_mean(door_right);
    left_distance = get_mean(door_left);
  } else return false;

  double posible_door_lenght = right_distance + left_distance;

  if ( (sizeof(outer_right) > 0) && (sizeof(outer_left) > 0) ){
    outer_right_distance = get_mean(outer_right);
    outer_left_distance = get_mean(outer_left);
  } else return false;

  double outer_distance = outer_right_distance + outer_left_distance;

  printf("\n\n %lf --| %lf |-- %lf \t DOOR", left_distance, posible_door_lenght, right_distance);
  printf("\n %lf --| %lf |-- %lf \t AMBIENT \n\n", outer_left_distance, outer_distance, outer_right_distance);

  
  if ( (outer_right.size()>door_right.size() && right_distance > 0) && (outer_left.size()>door_left.size() && left_distance > 0) ){   //Makes sure that there's wall in both sides
    for(int i=0; i<door_size.size(); i++){

      if ( ((door_size[i]+door_offset[i]) > posible_door_lenght) && (posible_door_lenght > (door_size[i]-door_offset[i])) ) {
        if( (outer_distance - posible_door_lenght) > door_wall_clearance[i]){
          //get_door_width(door_right, anlge_right);
          return true;
        } 
      }
    }
  }  
  
  return false;
}

void get_laser_data(const sensor_msgs::LaserScan::ConstPtr& msg){

  double max_range = msg->range_max;
  int N = (msg->angle_max - msg->angle_min)/msg->angle_increment; //Get number of readings
  std::vector<double> data;
  std::vector<geometry_msgs::Point> points;
  std::vector<double> angles;

  angles.reserve(N); 
  points.reserve(N);
  data.reserve(N); 
  
  for (int i = 0; i < N; i++){
    data[i] = msg->ranges[i];       //saving distances
  }

  calculate_angles( (msg->angle_max - msg->angle_min)*180.0/pi, N, angles);

  bool is_door = detect_door(angles, data, N, (msg->angle_max - msg->angle_min));
  std_msgs::Bool door;
  door.data = is_door;

  if (is_door) {ROS_INFO("\n\nIs a door!\n");}
  door_pub_.publish(door);
}

int main (int argc, char **argv){
  
  ros::init(argc, argv, "topo_detection");
  ros::NodeHandle n;

  ros::Subscriber lasertopic = n.subscribe<sensor_msgs::LaserScan>("/p3dx/laser/scan", 1, get_laser_data);
  door_pub_ = n.advertise<std_msgs::Bool>("topomapping/is_door", 2, true);
  ros::Rate looprate(4);

  while (ros::ok()){
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
} 