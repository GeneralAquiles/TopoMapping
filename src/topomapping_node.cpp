
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

waypoint last_waypoint;
waypoint last_detection;
waypoint last_door_detection;

int last_ID=0;
int actual_room;
//#############################################

/**
 * @brief Translates rotation in quaternion to NSOE coordinates
 * 
 * @param this_orientation geometry_msgs::Quaternion 
 * @return char - (N,S,O,E) 
 */
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

/**
 * @brief Returns a different integger each time it is called.
 * 
 * @return int 
 */
int generate_unique_ID(){
  last_ID+=1;
  return last_ID;
}

/**
 * @brief Get the room position in plant array
 * 
 * @param room_id Unique_id from desired room
 * @return int - Position in array. Returns -1 if not found
 */
int get_room_position(int room_id){
  for (int i = 0; i<plant.size(); i++){
    if (plant[i].unique_id == room_id) return i;
  }
  return -1;
}

/**
 * @brief Get the door position in plant array
 * 
 * @param room_id Unique_id from desired room
 * @return int - Position in array. Returns -1 if not found
 */
int get_door_position(int door_id){
  for (int i = 0; i<plant.size(); i++){
    if (doors[i].unique_id == door_id) return i;
  }
  return -1;
}

/**
 * @brief Get the closest point object
 * 
 * @param new_point Point object
 * @param waypoints Array of points to check with
 * @return double 
 */
double get_closest_point(waypoint new_point, std::vector<waypoint> waypoints){

  double closest_dist = sqrt( pow(new_point.position.x, 2) + pow(new_point.position.y, 2) );

  for(int i = 0; i<waypoints.size(); i++){
    double dist = sqrt( pow(new_point.position.x - waypoints[i].position.x, 2) + pow(new_point.position.y - waypoints[i].position.y, 2) );
    if (dist < closest_dist) closest_dist = dist;
  }
  return closest_dist;
}

/**
 * @brief Get the distance to object
 * 
 * @param new_point 
 * @param old_point 
 * @return double - Distance in module
 */
double get_distance(waypoint new_point, waypoint old_point){

  double closest_dist = sqrt( pow(new_point.position.x - old_point.position.x, 2) + pow(new_point.position.y - old_point.position.y, 2) );

  return closest_dist;
}

/**
 * @brief Get the door position in array.
 * 
 * @return int 
 */
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

int get_door_by_link(int link_ID){
  for (int i = 0; i < doors.size(); i++){
    
  }
}


int is_in_another_room(waypoint new_point){

  if ( get_distance(new_point, last_door_detection) > 0.9){ //This way we get reed of the las waypoint of the previous
    
    for (int i = 0; i < plant.size(); i++){
      if (plant[i].unique_id != actual_room){ //Avoids reading its own waypoints
        
        double closest_point = get_closest_point(new_point, plant[i].waypoints);
        if ( (closest_point < 0.9)  ) return plant[i].unique_id;
      }
    }
  }

  return -1;
}

/**
 * @brief Returns true if current direccion is the oposite to the last
 * 
 * @param current_dir 
 * @param last_dir 
 * @return true 
 * @return false 
 */
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

/**
 * @brief Returns wheter the passed point already exists
 * 
 * @param new_door 
 * @return true 
 * @return false 
 */
bool already_present(waypoint new_door){
  double distance = get_closest_point(new_door, doors); 
  if(distance > 0.75) return false;
  else return true;
}

/**
 * @brief Returns the ID of the room at the other side of the door.
 * 
 * @param door Door that is being crossed
 * @param this_room_id Room where you come from
 * @return int 
 */
int switching_to(waypoint door, int this_room_id){

  for(int i = 0; i<door.connections.size(); i++){
    if (door.connections[i].connected_to != this_room_id) return door.connections[i].connected_to;
  }
}



/**
 * @brief Creates a new room 
 * 
 */
void create_new_room(){
  Room new_room;
  connection link;
  
  int this_room = get_room_position(actual_room);

  if(this_room != -1) {

    link.connected_to = new_room.unique_id;
    plant[this_room].add_connection(link);
    doors[doors.size()-1].connections.push_back(link);

    link.connected_to = plant[this_room].unique_id;
    new_room.add_connection(link);
    doors[doors.size()-1].connections.push_back(link);

    plant[this_room].doors_id.push_back(doors[doors.size()-1].unique_id);
    new_room.doors_id.push_back(doors[doors.size()-1].unique_id);
  }

  plant.push_back(new_room);
  actual_room = new_room.unique_id;
}

/**
 * @brief Create a new door object
 * 
 * @return true If new door was created
 * @return false 
 */
bool create_new_door(){
  waypoint new_door;
  new_door.position = actual_position;

  if(!already_present(new_door)) {
    doors.push_back(new_door);
    return true;
  }
  else return false;
}

void change_room(){
  int i = get_closest_door();
  actual_room = switching_to(doors[i], actual_room);
}

/**
 * @brief Replaces door connection with a new one
 * 
 * @param door_id 
 * @param old_link_id 
 * @param new_link_id 
 * @return true - Successfully conected
 * @return false - Noy successfully conected
 */
bool change_door_link(int door_id, int old_link_id, int new_link_id){

  for (int i=0; i< doors[get_door_position(door_id)].connections.size(); i++){
    if (doors[get_door_position(door_id)].connections[i].connected_to == old_link_id){
      doors[get_door_position(door_id)].connections[i].connected_to = new_link_id;
      return true;
    }
  }

  std::cout << "[ERROR]: could not found door link " << old_link_id << std::endl;
  return false;
}

//void add_door_to_room(waypoint door, int room_id){}

void add_point_to_room(waypoint new_point, int room_id){
  int i = get_room_position(room_id);
  plant[i].waypoints.push_back(new_point);
}

/**
 * @brief Copies all waypoint from one room to another.
 * 
 * @param room__origin Room_ID where the waypoints come from.
 * @param room_destiny Room_ID where the points will be stored.
 */
void assing_points_to_room(int room__origin, int room_destiny){

  for (int i = 0; i < plant[get_room_position(room__origin)].waypoints.size(); i++){
    add_point_to_room(plant[get_room_position(room__origin)].waypoints[i], room_destiny);
  }
}

void delete_room(int room_id){
  plant.erase (plant.begin()+get_room_position(room_id));
}

rgb_color random_rgb_color()
{
  double rgb [3];
  rgb_color color;

  for(int i = 0; i < 3; i++)
  {
    rgb[i] = (0.8/(rand()%8)) + 0.1;
  }
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];

  return color;
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
    door_points.scale.y = 0.2;
    
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
}

void door_manager(const std_msgs::Bool::ConstPtr& msg){
  bool is_door = msg->data;
  waypoint new_point;
  new_point.is_door = is_door;
  new_point.position = actual_position;
  new_point.orientation = quaternion_to_orientation(actual_orientation);

  if(plant.size() == 0) create_new_room();
  
  if ( (new_point.is_door) && ( (get_distance(new_point, last_door_detection) > 0.5) || (is_oposite_direction(new_point.orientation, last_door_detection.orientation)) ) ) {
    if (create_new_door()) {
      create_new_room ();
      bool done;
      std::cout<<"\n It's a door! "<<std::endl;
      std::cout<<" Moving into new room: "<< actual_room <<std::endl;
      std::cout<<" Rooms found: "<< plant.size() << std::endl;
    }
    else {
      change_room();
      std::cout<<"\n\n\n\n Door crossed, changing to room "<< actual_room <<std::endl;

    }

    last_door_detection = new_point;

    topo_map_print();
  }
  else if (!new_point.is_door && (get_closest_point(new_point, plant[get_room_position(actual_room)].waypoints) > 1.2) )
  {
    // aqui se añaden los puntos a las habitaciones
    int old_room = is_in_another_room(new_point);
    if (old_room != -1){
      //update waypoints
      assing_points_to_room(actual_room, old_room);
      //update doors
      for (int i = 0; i < plant[get_room_position(actual_room)].doors_id.size(); i++){        
        change_door_link( plant[get_room_position(actual_room)].doors_id[i], actual_room, old_room);  //modifying door link
        plant[get_room_position(old_room)].doors_id.push_back(plant[get_room_position(actual_room)].doors_id[i]); //assigning door to old room
      }
      //update room links
      for (int i = 0; i < plant[get_room_position(actual_room)].connections.size(); i++){

        connection this_id = plant[get_room_position(actual_room)].connections[i];
        plant[get_room_position(old_room)].add_connection(this_id); //assigning links to old room
        plant[get_room_position(this_id.connected_to)].change_connection(actual_room, old_room);
      }

      delete_room(actual_room);
      std::cout << "\n [INFO]: Deleting room " << actual_room << std::endl;
   
      actual_room = old_room;
      last_detection = new_point;

      std::cout << " [INFO]: Reassingning data to room " << actual_room << std::endl;
    }
    else{
      add_point_to_room(new_point, actual_room);
      last_detection = new_point;
      std::cout << "\nPoint added to "<< actual_room << " room id"std::endl;
      
      srv.request;
      std_msgs::Bool upload_image;
      upload_image.data = true;
      google_AI.publish(upload_image);      
      if(image_save.call(srv)) std::cout<<"\n Image taken! "<<std::endl;

      else  std::cout<<"\n[WARNING]: No response recieved... Check Image_saver has been launched"<<std::endl;
    }

    topo_map_print();
  }
  
}

/**
 * @brief Get the pose data object.
 * 
 * @param msg RosTopic where the data is extracted from.
 */
void get_pose_data(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  geometry_msgs::PoseWithCovarianceStamped actual;

  actual.pose = msg->pose;
  actual_position = actual.pose.pose.position;
  actual_orientation = actual.pose.pose.orientation;
}

void assing_name(const topomapping::probability_array::ConstPtr& msg){
  std::cout << "\n[INFO]: Estimation recieved:" << std::endl;
  std::cout << "____________________________" << std::endl;
  topomapping::probability_array recieved_probabilities;
  recieved_probabilities.items = msg->items;

  for(int i = 0; i < recieved_probabilities.items.size(); i++){
    std::cout << recieved_probabilities.items[i] << std::endl;
  }
  std::cout << "____________________________" << std::endl;

  plant[get_room_position(actual_room)].add_new_estimation(recieved_probabilities);
  plant[get_room_position(actual_room)].calculate_room_name();
}

/*
void nodepool(){
  listen to isdoor

  if is door --> plant.append(new room)
}
*/
int main (int argc, char **argv){
  
  last_detection.position.x=0;
  last_detection.position.y=0;
  ros::init(argc, argv, "toponode");
  ros::NodeHandle n;

  ros::Subscriber posetopic = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, get_pose_data);
  ros::Subscriber door_topic = n.subscribe<std_msgs::Bool>("topomapping/is_door", 1, door_manager);
  ros::Subscriber porbability_topic = n.subscribe<topomapping::probability_array>("topomapping/room_type", 1, assing_name);

  topo_pub = n.advertise<visualization_msgs::Marker>("topomapping/topo_map", 10);
  google_AI = n.advertise<std_msgs::Bool>("topomapping/upload_photo", 10);
  image_save = n.serviceClient<std_srvs::Empty>("/image_saver/save");

  ros::Rate looprate(4);

  while (ros::ok()){
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
} 

Room::Room(){
      unique_id = generate_unique_ID();
      name = std::to_string(unique_id);
}

waypoint::waypoint(){
      unique_id = generate_unique_ID();
}

void Room::add_connection(connection new_conection){

  connections.push_back(new_conection);
}

void Room::change_connection(int old_id, int new_id){
  for ( int i = 0; i < connections.size(); i++){
    if ( connections[i].connected_to == old_id) connections[i].connected_to = new_id;
  }
}

void Room::add_new_estimation(topomapping::probability_array new_estimation){
  bool add_it = false;
  for (int item = 0; item < new_estimation.items.size(); item++){
   if(new_estimation.items[item].prob > 0){
     add = true;
     break
   }
  }
  if (add_it) all_received_estimations.push_back(new_estimation);
}

void Room::calculate_room_name(){
  if (all_received_estimations.size() > 0){
    
    std::vector<topomapping::probability_element> calculated_probabilities;

    //Calculate average provability for each candidate
    for (int candidate = 0; candidate < all_received_estimations[0].items.size(); candidate++){

      topomapping::probability_element calculated_candidate;
      double accumulated_prob = 0;

      //Loop trough all stimations rcieved
      for (int estimation = 0; estimation < all_received_estimations.size(); estimation++){
        accumulated_prob += all_received_estimations[estimation].items[candidate].prob;
      }

      calculated_candidate.prob = accumulated_prob/all_received_estimations.size();
      calculated_candidate.name = all_received_estimations[0].items[candidate].name;

      calculated_probabilities.push_back(calculated_candidate);
    }

    double last_prob = 0;
    int best_candidate = -1;
    for (int candidate = 0; candidate < calculated_probabilities.size(); candidate++){
      if (calculated_probabilities[candidate].prob > last_prob){
        last_prob = calculated_probabilities[candidate].prob;
        best_candidate = candidate;
      }
    }

    if (best_candidate != -1){
      if (calculated_probabilities[best_candidate].prob > 0) name = calculated_probabilities[best_candidate].name + " (" +std::to_string(calculated_probabilities[best_candidate].prob) + ")";
      else name = "Unknown (id: " + std::to_string(unique_id) + ")";
    }
    else name = "Unknown (id: " + std::to_string(unique_id) + ")";
  }
  else name = "Unknown (id: " + std::to_string(unique_id) + ")";
}

geometry_msgs::Point  Room::get_central_point(){
  
    central_point.x = 0;
    central_point.y = 0;

  if ( waypoints.size() > 0){
    for ( int i = 0; i<waypoints.size(); i++){
      central_point.x += waypoints[i].position.x;
      central_point.y += waypoints[i].position.y;
    }

    central_point.x = central_point.x/(waypoints.size()-0);
    central_point.y = central_point.y/(waypoints.size()-0);
    central_point.z = 2;

    return central_point;
  }
}
