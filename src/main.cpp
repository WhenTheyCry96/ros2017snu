// state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>
#include <gazebo_msgs/ModelStates.h>   // visualize
#include <gazebo_msgs/SetModelState.h> // visualize
#include <gazebo_msgs/SpawnModel.h>    // visualize
#include <project2/pid.h>
#include <project2/rrtTree.h>
#include <pwd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

// map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

// parameters we should adjust : K, margin, MaxStep
int margin = 5;           // originally 6
int K = 1000;             // originally 500
double MaxStep = 2.3;     // originally 2
int waypoint_margin = 20; // originally 24

// way points
std::vector<point> waypoints;

// path
std::vector<traj> path_RRT;

// robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

// park
PID pid_ctrl;
gazebo_msgs::ModelStatesConstPtr model_states;

// FSM state
int state;

// function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

static double angle_bound(double ang) {
  while (ang > M_PI)
    ang -= 2 * M_PI;
  while (ang < -M_PI)
    ang += 2 * M_PI;
  return ang;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "slam_main");
  ros::NodeHandle n;

  // Initialize topics
  ros::Publisher cmd_vel_pub =
      n.advertise<ackermann_msgs::AckermannDriveStamped>(
          "/vesc/high_level/ackermann_cmd_mux/input/nav_0", 1);
  ros::Subscriber gazebo_pose_sub =
      n.subscribe("/amcl_pose", 100, callback_state);

  // visualize RRT
  ros::ServiceClient gazebo_spawn =
      n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  ros::ServiceClient gazebo_set =
      n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  printf("Initialize topics\n");

  // FSM
  state = INIT;
  bool running = true;
  ros::Rate control_rate(60);
  // park
  int look_ahead_idx;

  while (running) {
    switch (state) {
    case INIT: {
      // park
      look_ahead_idx = 0;
      // Load Map
      char *user = getpwuid(getuid())->pw_name;
      cv::Mat map_org =
          cv::imread((std::string("/home/") + std::string(user) +
                      std::string("/catkin_ws/src/final_project/src/final.pgm"))
                         .c_str(),
                     CV_LOAD_IMAGE_GRAYSCALE);

      cv::transpose(map_org, map);
      cv::flip(map, map, 1);

      map_y_range = map.cols;
      map_x_range = map.rows;
      map_origin_x = map_x_range / 2.0 - 0.5;
      map_origin_y = map_y_range / 2.0 - 0.5;
      world_x_min = -4.7;
      world_x_max = 4.7;
      world_y_min = -10.2;
      world_y_max = 10.2;
      res = 0.05;
      printf("Load map\n");

      if (!map.data) // Check for invalid input
      {
        printf("Could not open or find the image\n");
        return -1;
      }
      state = PATH_PLANNING;
    } break;

    case PATH_PLANNING:

      // Set Way Points
      set_waypoints();
      printf("Set way points\n");

      // RRT
      generate_path_RRT();
      printf("Generate RRT\n");

      // visualize
      printf("path size : %d\n", path_RRT.size());
      for (int i = 0; i < path_RRT.size(); i++) {

        gazebo_msgs::SpawnModel model;
        model.request.model_xml =
            std::string("<robot name=\"simple_ball\">") +
            std::string("<static>true</static>") +
            std::string("<link name=\"ball\">") + std::string("<inertial>") +
            std::string("<mass value=\"1.0\" />") +
            std::string("<origin xyz=\"0 0 0\" />") +
            std::string("<inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  "
                        "iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />") +
            std::string("</inertial>") + std::string("<visual>") +
            std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
            std::string("<geometry>") +
            std::string("<sphere radius=\"0.09\"/>") +
            std::string("</geometry>") + std::string("</visual>") +
            std::string("<collision>") +
            std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
            std::string("<geometry>") +
            std::string("<sphere radius=\"0.09\"/>") +
            std::string("</geometry>") + std::string("</collision>") +
            std::string("</link>") +
            std::string("<gazebo reference=\"ball\">") +
            std::string("<mu1>10</mu1>") + std::string("<mu2>10</mu2>") +
            std::string("<material>Gazebo/Blue</material>") +
            std::string("<turnGravityOff>true</turnGravityOff>") +
            std::string("</gazebo>") + std::string("</robot>");

        std::ostringstream ball_name;
        ball_name << i;
        model.request.model_name = ball_name.str();
        model.request.reference_frame = "world";
        model.request.initial_pose.position.x = path_RRT[i].x;
        model.request.initial_pose.position.y = path_RRT[i].y;
        model.request.initial_pose.position.z = 0.7;
        model.request.initial_pose.orientation.w = 0.0;
        model.request.initial_pose.orientation.x = 0.0;
        model.request.initial_pose.orientation.y = 0.0;
        model.request.initial_pose.orientation.z = 0.0;
        gazebo_spawn.call(model);

        ros::spinOnce();
      }
      ros::Rate(0.33).sleep();
      printf("Initialize ROBOT\n");
      state = RUNNING;

    case RUNNING: {
      // TODO 1
      if (std::sqrt(std::pow(robot_pose.x - path_RRT[look_ahead_idx + 1].x, 2) +
                    std::pow(robot_pose.y - path_RRT[look_ahead_idx + 1].y,
                             2)) < 0.2) {
        std::cout << "Passed " << look_ahead_idx + 1 << std::endl;
        // park
        if (++look_ahead_idx == path_RRT.size() - 2) {
          std::cout << "1st Goal Passed" << std::endl;

        } else if (++look_ahead_idx == path_RRT.size() - 1) {
          std::cout << "2nd Goal Passed" << std::endl;
          state = FINISH;
          break;
        }
      }

      float vel = 2.5f;

      if (look_ahead_idx > 9) { // after finish 1cycle
        pid_ctrl.after_outer(1);
        vel = 1.5f;
      }

      float alpha = pid_ctrl.get_control(robot_pose, path_RRT[look_ahead_idx],
                                         path_RRT[look_ahead_idx + 1]);

      float distsq =
          std::pow(robot_pose.x - path_RRT[look_ahead_idx + 1].x, 2) +
          std::pow(robot_pose.y - path_RRT[look_ahead_idx + 1].y, 2);
      pid_ctrl.clear_error_sum();
      setcmdvel(vel, alpha);
      cmd_vel_pub.publish(cmd);
      ros::spinOnce();
      control_rate.sleep();

    } break;

    case FINISH: {
      setcmdvel(0, 0);
      cmd_vel_pub.publish(cmd);
      std::cout << "---------------------------------------------" << std::endl;
      std::cout << "FINISH" << std::endl;
      std::cout << "---------------------------------------------" << std::endl;
      running = false;
      ros::spinOnce();
      control_rate.sleep();
    } break;
    default: {
      // nothing
    } break;

      return 0;
    }
  }
}

void setcmdvel(double vel, double deg) {
  cmd.drive.speed = vel;
  cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs) {
  robot_pose.x = msgs->pose.pose.position.x;
  robot_pose.y = msgs->pose.pose.position.y;
  robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
  // printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints() {
  waypoints.clear();
  std::srand(std::time(NULL));

  cv::Mat map_margin = map.clone();
  int jSize = map.cols; // the number of columns
  int iSize = map.rows; // the number of rows

  for (int i = 0; i < iSize; i++) {
    for (int j = 0; j < jSize; j++) {
      if (map.at<uchar>(i, j) < 125) {
        for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
          for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
            if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
              map_margin.at<uchar>(k, l) = 0;
            }
          }
        }
      }
    }
  }
  point waypoint_candid[15];
  // Starting point. (Fixed)
  waypoint_candid[0].x = -3.5;
  waypoint_candid[0].y = 8.5;
  waypoint_candid[0].th = 0;
  // TODO 2
  // Set your own waypoints.
  // The car should turn around the outer track once, and come back to the
  // starting point.
  // This is an example.
  waypoint_candid[1].x = 0.0;
  waypoint_candid[1].y = 9.0;
  waypoint_candid[2].x = 2.3; // 2.2
  waypoint_candid[2].y = 9.0; // 8.6
  waypoint_candid[3].x = 3.7;
  waypoint_candid[3].y = 5.5;
  waypoint_candid[4].x = 4.0;
  waypoint_candid[4].y = -3.0;

  waypoint_candid[5].x = 3.5;
  waypoint_candid[5].y = -6.6;
  waypoint_candid[6].x = 0.0;
  waypoint_candid[6].y = -9.0;
  waypoint_candid[7].x = -2.4;
  waypoint_candid[7].y = -8.4;

  waypoint_candid[8].x = -3.8;
  waypoint_candid[8].y = -4.2;
  waypoint_candid[9].x = -4.0;
  waypoint_candid[9].y = 2.5;
  waypoint_candid[9].th = std::atan2(6.0, 0.5);

  waypoint_candid[10].x = -3.5;
  waypoint_candid[10].y = 8.5;

  // world_x_max : 4.7 world_y_max : 10.2
  /*
  double world_x[] = {3 / 4 * world_x_max, 3 / 4 * world_x_max, -world_x_max,
                      -world_x_max};
  double world_y[] = {};

  double rX, rY;
  double world_partX = 2;
  double world_partY = 2.5;
  for (int i = 0; i < 4; i++) {
    do {
      const double rand1 = rand() / static_cast<double>(RAND_MAX);
      const double rand2 = rand() / static_cast<double>(RAND_MAX);
      rX = world_x[i] + rand1 *;
      rY = world_y[i] + rand2 *;
      rX = rX / res + map_origin_x;
      rY = rY / res + map_origin_y;
    } while (map_margin.at<uchar>(rX, rY) < 155);
    waypoint_candid[i + 1].x = res * (rX - map_origin_x);
    waypoint_candid[i + 1].y = res * (rY - map_origin_y);
  }
  */
  // Waypoints for arbitrary goal points.
  // TA will change this part before scoring.
  // This is an example.
  waypoint_candid[11].x = 1.5;
  waypoint_candid[11].y = 1.5;
  waypoint_candid[12].x = -2;
  waypoint_candid[12].y = -9.0;

  // int order[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
  int order[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  int order_size = 13;

  for (int i = 0; i < order_size; i++) {
    waypoints.push_back(waypoint_candid[order[i]]);
  }
}

void generate_path_RRT() {
  // TODO 1
  int success, i;

  do {
    set_waypoints();
    path_RRT.clear();
    // park

    for (i = 0; i < 9; i++) { // size -3

      traj tmp;
      tmp.x = waypoints[i].x;
      tmp.y = waypoints[i].y;
      tmp.th = std::atan2(waypoints[i + 1].y - waypoints[i].y,
                          waypoints[i + 1].x - waypoints[i].x);
      // angle_bound(tmp.th);
      path_RRT.insert(path_RRT.end(), tmp);
      std::cout << "Found Path " << i << " pose : (" << path_RRT[i].x << " ,"
                << path_RRT[i].y << ") Theta : " << path_RRT[i].th << std::endl;
    }
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Path for Outer cycle Found" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;

    for (i = 9; i < waypoints.size() - 1; i++) { // 11
      rrtTree tree(waypoints[i], waypoints[i + 1], map, map_origin_x,
                   map_origin_y, res, margin);
      int count = 0;

      do {
        success = tree.generateRRT(world_x_max, world_x_min, world_y_max,
                                   world_y_min, K, MaxStep);

      } while (success != 1 && count++ < 14); // originally 9
      if (count == 15)                        // originally 10
        break;

      std::vector<traj> path(tree.backtracking_traj());
      path_RRT.insert(path_RRT.end(), path.rbegin(), path.rend());
      waypoints[i + 1].th = path.front().th;
      std::cout << "Found Path " << i << " pose : (" << path_RRT[i].x << " ,"
                << path_RRT[i].y << ") Theta : " << path_RRT[i].th << std::endl;
    }

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Path for Goals Founded" << std::endl;
    std::cout << "----------------- ----------------------------" << std::endl;
  } while (i != waypoints.size() - 1);
}
