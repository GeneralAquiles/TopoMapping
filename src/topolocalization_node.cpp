
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

// Create a Car class with some attributes
class rgb_color {
  public:
    double r,g,b;
    double a = 1.0;
};

class connection {
  public:
    double connected_to;
};

class waypoint {
  public:
    geometry_msgs::Point position;
    int unique_id;
    std::vector<connection> connections;
    char orientation;
    bool is_door;

    waypoint();
};

class Room {
  public:
    geometry_msgs::Point central_point;
    std::vector<waypoint> waypoints;   
    std::vector<connection> connections;
    std::vector<int> doors_id;
    std::vector<topomapping::probability_array> all_received_estimations;
    std::string name;
    int unique_id;

    Room();
    void add_connection(connection new_conection);
    geometry_msgs::Point get_central_point();
    void change_connection(int old_ID, int new_id);
    void add_new_estimation(topomapping::probability_array new_estimation);
    void calculate_room_name();
    int  closest_point_id(waypoint new_point);
    waypoint get_point(int point_id);
};

//#############Internal Variables##############
std::vector<Room> plant;
std::vector<waypoint> doors;
std::vector<rgb_color> room_colors;

std_srvs::Empty srv;

ros::Publisher topo_pub;
ros::Publisher google_AI;
ros::ServiceClient image_save;

geometry_msgs::Point actual_position;
geometry_msgs::Quaternion actual_orientation;

topomapping::probability_array probability;

waypoint actual_waypoint;
waypoint last_door_detection;

int last_ID=0;
int actual_room = -1;
int actual_waypoint_id = -1;
//#############################################

char quaternion_to_orientation( geometry_msgs::Quaternion this_orientation){
    //  |z| ~ 0 --> alpha = 0     |   w ~ 1 --> alpha = 0
    //  |z| ~ 1 --> alpha = 180   |   w ~ 0 --> alpha = 1
    //  z > 0 --> 180>alpha>0
    //  z < 0 --> 360>alpha>180

    // orientation N - S - E - O     
    // E --> alpha = 0
    // O --> alpha = 180
    // N --> alpha = 90 
    // S --> alpha = -90

  char orientation;
  double z = this_orientation.z;
  double w = this_orientation.w;

  if ( w > 0.75 ) orientation = 'E';
  else if ( w < 0.25 ) orientation = 'O';
  else if ( z > 0) orientation = 'N';
  else if ( z < 0) orientation = 'S';

  return orientation;
}

int get_room_position(int room_id){
  for (int i = 0; i<plant.size(); i++){
    if (plant[i].unique_id == room_id) return i;
  }
  return -1;
}

int get_door_position(int door_id){
  for (int i = 0; i<plant.size(); i++){
    if (doors[i].unique_id == door_id) return i;
  }
  return -1;
}

int get_closest_door(){
  double closest_dist = sqrt( pow(actual_position.x, 2) + pow(actual_position.y, 2) );

  int position;
  for(int i = 0; i<doors.size(); i++){
    double dist = sqrt( pow(actual_position.x - doors[i].position.x, 2) + pow(actual_position.y - doors[i].position.y, 2) );
    if (dist < closest_dist){
      closest_dist = dist;
      position = i;
    } 
  }

  return position;
}

double get_closest_point_dist(waypoint new_point, std::vector<waypoint> waypoints){

  double closest_dist = sqrt( pow(new_point.position.x, 2) + pow(new_point.position.y, 2) );

  for(int i = 0; i<waypoints.size(); i++){
    double dist = sqrt( pow(new_point.position.x - waypoints[i].position.x, 2) + pow(new_point.position.y - waypoints[i].position.y, 2) );
    if (dist < closest_dist) closest_dist = dist;
  }
  return closest_dist;
}

double get_distance(waypoint new_point, waypoint old_point){

  double closest_dist = sqrt( pow(new_point.position.x - old_point.position.x, 2) + pow(new_point.position.y - old_point.position.y, 2) );

  return closest_dist;
}

int is_in_another_room(waypoint new_point){

  if ( get_distance(new_point, last_door_detection) > 0.9){ //This way we get reed of the las waypoint of the previous
    
    for (int i = 0; i < plant.size(); i++){
      if (plant[i].unique_id != actual_room){ //Avoids reading its own waypoints
        
        double closest_point = get_closest_point_dist(new_point, plant[i].waypoints);
        if ( (closest_point < 0.9)  ) return plant[i].unique_id;
      }
    }
  }

  return -1;
}

int relocate_position(waypoint new_point){ 
 
  double closest_dist;
  double last_closest_dist = 99999;
  int closest_room = -1;
  for (int i = 0; i < plant.size(); i++){
    if (plant[i].unique_id != actual_room){ //Avoids reading its own waypoints
      
      double closest_dist = get_closest_point_dist(new_point, plant[i].waypoints);
      if ( (closest_dist < last_closest_dist)  ) {
        last_closest_dist = closest_dist;
        closest_room = plant[i].unique_id;
      }
    }
  }
  return closest_room;
}

bool is_oposite_direction(char current_dir, char last_dir){

  switch (current_dir)
  {
  case 'N':
    if (last_dir == 'S') return true;
    else return false;
    break;
  case 'S':
    if (last_dir == 'N') return true;
    else return false;
    break;
  case 'E':
    if (last_dir == 'O') return true;
    else return false;
    break;
  case 'O':
    if (last_dir == 'E') return true;
    else return false;
    break;
  
  default:
    std::cout << "\n [WARNING]: Something strange happened with orientatinons." << std::endl;
    break;
  }
  return false;

}

int switching_to(waypoint door, int this_room_id){

  for(int i = 0; i<door.connections.size(); i++){
    if (door.connections[i].connected_to != this_room_id) return door.connections[i].connected_to;
  }
}

void change_room(){
  int i = get_closest_door();
  actual_room = switching_to(doors[i], actual_room);
}

void topo_map_print(){
      
  for( int each_room = 0; each_room < plant.size(); each_room++){
    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = each_room;

    points.type = visualization_msgs::Marker::SPHERE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.scale.y = 0.2;

    if(each_room >= room_colors.size()){
      room_colors.push_back(random_rgb_color());
    }

    points.color.a = room_colors[each_room].a;
    points.color.r = room_colors[each_room].r;
    points.color.g = room_colors[each_room].g;
    points.color.b = room_colors[each_room].b;

    points.points.clear();

    for( int i = 0; i < plant[each_room].waypoints.size(); i++ ){
      geometry_msgs::Point p;
      p = plant[each_room].waypoints[i].position;
      p.z = 1;

      points.points.push_back(p);
    }

    topo_pub.publish(points);
  }
    



    visualization_msgs::Marker door_points;
    door_points.header.frame_id = "/map";
    door_points.header.stamp = ros::Time::now();
    door_points.ns = "points_and_lines";
    door_points.action = visualization_msgs::Marker::ADD;
    door_points.pose.orientation.w = 1.0;
    door_points.id = plant.size()+1;
    door_points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    door_points.scale.x = 0.2;
    door_points.scale.y = 0.5;
    
    door_points.color.a = 1.0;
    door_points.color.r = 1.0;
    door_points.color.g = 0.0;
    door_points.color.b = 0.0;      

    for( int i = 0; i < doors.size(); i++ ){
      geometry_msgs::Point p;
      p = doors[i].position;
      p.z = 1;

      door_points.points.push_back(p);
    }
    topo_pub.publish(door_points);





    visualization_msgs::Marker room_names;
    room_names.header.frame_id = "/map";
    room_names.header.stamp = ros::Time::now();
    room_names.ns = "points_and_lines";
    room_names.action = visualization_msgs::Marker::ADD;
    room_names.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    room_names.scale.z = 0.5;

    // Points are green
    room_names.color.a = 1.0;
    room_names.color.r = 1.0;
    room_names.color.g = 0.0;
    room_names.color.b = 0.0;    

    for( int i = 0; i < plant.size(); i++ ){
      if ( plant[i].waypoints.size() > 0){
        room_names.id = plant.size()+2+i;

        room_names.pose.position = plant[i].get_central_point();
        room_names.pose.orientation.w = 1.0;

        room_names.text = plant[i].name;
        topo_pub.publish(room_names);
      }
    }

    visualization_msgs::Marker robot;
    robot.header.frame_id = "/map";
    robot.header.stamp = ros::Time::now();
    robot.ns = "points_and_lines";
    robot.action = visualization_msgs::Marker::ADD;
    robot.type = visualization_msgs::Marker::cube;
    robot.scale.z = 0.5;
    robot.scale.x = 0.3;
    robot.id =  2*plant.size()+2;
    robot.color.a = 1.0;
    robot.color.r = 0.0;
    robot.color.g = 1.0;
    robot.color.b = 0.0;    
    robot.pose.position = actual_waypoint.position;
    robot.pose.position.z = 1;
    topo_pub.publish(robot);

}

void localize(const std_msgs::Bool::ConstPtr& msg){
  bool is_door = msg->data;
  waypoint new_point;
  new_point.is_door = is_door;
  new_point.position = actual_position;
  new_point.orientation = quaternion_to_orientation(actual_orientation);
  
  if (actual_room == -1){
    actual_room = relocate_position(new_point);
    actual_waypoint_id = plant[get_room_position(actual_room)].room::closest_point_id(new_point);
  }

  if ( (new_point.is_door) && ( (get_distance(new_point, last_door_detection) > 0.5) || (is_oposite_direction(new_point.orientation, last_door_detection.orientation)) ) ) {
    change_room();
    std::cout<<"\n\n\n\n Door crossed, changing to room "<< actual_room <<std::endl;

    last_door_detection = new_point;
    actual_waypoint = doors[get_closest_door];
    actual_waypoint_id = actual_waypoint.unique_id;
    topo_map_print();
  }
  else if (!new_point.is_door)
  {
    int posible_position_id = plant[get_room_position(actual_room)].room::closest_point_id(new_point);
    waypoint posible_position =  plant[get_room_position(actual_room)].room::get_point(posible_position_id);


    if (get_distance(new_point, posible_position) < 1.2) actual_waypoint_id = posible_position.unique_id;
    else {
      actual_room = relocate_position(new_point);
      actual_waypoint_id = plant[get_room_position(actual_room)].room::closest_point_id(new_point);
      actual_waypoint = plant[get_room_position(actual_room)].room::get_point(actual_waypoint_id);
    }

    topo_map_print();
  } 
}

void get_pose_data(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  geometry_msgs::PoseWithCovarianceStamped actual;

  actual.pose = msg->pose;
  actual_position = actual.pose.pose.position;
  actual_orientation = actual.pose.pose.orientation;
}

int main (int argc, char **argv){
  ros::init(argc, argv, "Localization");
  ros::NodeHandle n;

  ros::Subscriber posetopic = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, get_pose_data);
  ros::Subscriber door_topic = n.subscribe<std_msgs::Bool>("topomapping/is_door", 1, door_manager);

  topo_pub = n.advertise<visualization_msgs::Marker>("topomapping/topo_map", 10);
  ros::Rate looprate(4);

  while (ros::ok()){
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
} 

int  room::closest_point_id(waypoint new_point){
  double closest_dist = 99999;
  int closest_point = -1;

  for(int i = 0; i<waypoints.size(); i++){
    double dist = sqrt( pow(new_point.position.x - waypoints[i].position.x, 2) + pow(new_point.position.y - waypoints[i].position.y, 2) );
    if (dist < closest_dist){
      closest_dist = dist;
      closest_point = i;
    } 
  }
  return waypoints[closest_point].unique_id;
}

waypoint room::get_point(int point_id){
  for(int i = 0; i<waypoints.size(); i++){
    if (waypoints[i].unique_id == point_id) return waypoints[i];
  }
}