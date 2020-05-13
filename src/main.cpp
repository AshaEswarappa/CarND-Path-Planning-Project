  
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;

using namespace std;

/*start the car in center lane*/
//lane: 0 - left, 1- center, 2 -right */
int lane = 1;

/*set the reference velocity*/
double ref_vel = 0; //mph

/*set the lane width */
int lanewidth = 4;        

/*set the offset for increase or decrease of the velocity */
float offset = 0.224; //to meet the specification of requiremnet of project related to acceleration or jerk */

/*Maximu speed of the lane */
double MAX_SPEED = 49.6;

int main() 
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;


  ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /*set the size of previous path */ 
          int prev_path_size = previous_path_x.size();

          /*checks if the previous point is not epmty */
          if(prev_path_size > 0)
          {
            car_s  = end_path_s; /*previous points in s-coordinate */

          }

          /*declaration of other car lane */
          int car_lane = -1; 

          /* acceleration or deceleration offset information based on sensor fusion data */
          double speed_diff = 0;

          /* Boolean Signals to indicate if there is car in the front/left or right */ 
          bool car_left = false;

          bool car_right = false;

          bool car_front = false;

          /*set the safe distance to be maintained between cars*/
          int min_safe_dist = 30; // in meter

          /*Find reference velocity to be used */
          for(int i = 0; i <sensor_fusion.size(); i++)
          {
            //check car is in which laneane

            float d = sensor_fusion[i][6]; // Senosr fusion parameters are vetor of vectors

            /*check if the car is in the left, center or right lane respectively */
            if( d < lanewidth && d > 0)
            {
               car_lane = 0; // left lane
            }
            else if(d < 2*lanewidth && d > lanewidth)
            {
              car_lane = 1; // center lane

            }
            else
            {
              car_lane = 2; //right lane

            }
      
            double vx = sensor_fusion[i][3]; //velocity in x-direction
            double vy = sensor_fusion[i][4]; // velocity in y -direction

            double check_speed = sqrt (pow(vx,2) + pow(vy,2)); // speed of the car

            double check_car_s = sensor_fusion[i][5]; //provides information to check if the car is close to our car(end_path_s)

              /* using previous path projects the s values on car */
              check_car_s += ((double) prev_path_size * 0.02 *check_speed);

              /*Check if lane change is allowed or not */
              /*check if the car is in same lane*/
             if(car_lane == lane)
             {

                car_front |= ((check_car_s > car_s) && ((check_car_s - car_s) <= min_safe_dist));  /*check if the car is too close and checks the safe distance between EGO  vehicle and car in front*/
              

             }
             else if((car_lane - lane) == 1)
             {

                car_right |= ((car_s - min_safe_dist) < check_car_s && (car_s + min_safe_dist) > check_car_s); /*check if the car is close to the one on right lane before making lane change decision*/

             }

             else
             {

                car_left |= ((car_s - min_safe_dist) < check_car_s && (car_s + min_safe_dist) > check_car_s);  /*check if the car is close to the one on left lane before making lane change decision*/

             }

             
          }

            /*check if the car is in front */
            if (car_front) 
            { 
               /* if there is no car in left and ego vehicle lane is not a left lane then lane change left*/
              if (!car_left && lane > 0)
              {
               
                lane--; // decrement the curret lane value by 1 after lane change left

              } 
              else if (!car_right && lane != 2)
              {
                // if there is no car right and its not the right lane

                lane++; // increment the current lane value by 1 after lane change right
              } 
              else 
              {
                speed_diff -=  offset; // set the offset value for decreasing velocity in order to stay in same lane and avoid collision
              }

            }
            else 
            {
                // if we are no more on the center lane and has already done lane chane earlier
              if (lane != 1 ) 
              { 
                //check whether we are in left lane or right lane along with existance of neighbour car
                if ( ( lane == 0 && !car_right ) || ( lane == 2 && !car_left ) ) 
                {
                    lane = 1; // come to original position
                }

              }

              /*check if the reference velocity is less than the lane speed lmiit*/
              if ( ref_vel < MAX_SPEED) 
              {
                speed_diff += offset; /*add the offset as long as it has not reached the speed limit*/

              }

          }

    
          
          json msgJson;

     

         /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          /*create widely and  evenly spaced at 30m waypoint(x,y)*/
          vector<double> ptsx;
          vector<double> ptsy;

          /*set the Number of points in path */
          int NUM_POINTS = 50;

          //set the reference position as the car poistion at this point and this might change depending on the situation or not
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          /*if the previous path is empty use the car position as reference point*/
          if(prev_path_size < 2)
          {
            /*use the two points to form tangent to the car*/
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          else
          {

            /*if the path is not empty then choose the previous end points as reference points */
            ref_x = previous_path_x[prev_path_size -1];
            ref_y = previous_path_y[prev_path_size -1];

            double prev_ref_x = previous_path_x[prev_path_size -2];
            double prev_ref_y = previous_path_y[prev_path_size -2];

            ref_yaw = atan2((ref_y - prev_ref_y), (ref_x - prev_ref_x));

            /*use the two points which makes tangent to the previous path end points*/
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);

          }

          /*In frenet co-ordinates, add evenly spaced 30m points ahead of starting reference points*/
          vector<double> next_wp0 = getXY(car_s + 30, (2 + lanewidth*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2 + lanewidth*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2 + lanewidth*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp3 = getXY((car_s + 120), (2 + lanewidth*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          /*push the converted points back to the setpoints */

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsx.push_back(next_wp3[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          ptsy.push_back(next_wp3[1]);

          for(int i = 0; i < ptsx.size(); i++)
          {
            
            /*Transformation of co-odrinates to vehicle co-ordinates*/
            /*shift car reference angle to 0 degree*/

            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*(sin(0 - ref_yaw)));
            ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*(cos(0 - ref_yaw)));
            

          }


          /*Create a spline */
          tk::spline s;

          /*set the spline points*/
          s.set_points(ptsx, ptsy);

          /*define the actual points that we use for planner */
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i = 0; i < prev_path_size ; i++)
          {
            /* use the previous path points to generate new path instead of generating from beginning */
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
            
          }

          /*Split the spline points so that we could reach the desired reference velocits */
          double target_x = 30.0;
          double target_y = s(target_x); //gets the y-points for the defined x-points of spline
          double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

          double x_add_on = 0; /*needed for local tranformation */

          /*if the previous path size is less than desired number of points of path planner after filling it with previous points, here we will fill the remaining points*/
          for(int i = 1; i <= (NUM_POINTS - prev_path_size); i++)
          {
            /*check if the vehicle is in start poistion*/
            if(ref_vel == 0)
            {
              ref_vel = 2; //start with some initial velocity

            }
            
            ref_vel += speed_diff; // increase or decrease depending on the sension fusion information for collision check
              
            /*check if the reference velocity is above the speed limit*/
            if ( ref_vel > MAX_SPEED) 
            {
                ref_vel = MAX_SPEED; //limit the reference velocity to speed limit
            } 
            /*check if the velocity is less than the offset value */
            else if (ref_vel < offset) 
            {
                ref_vel = offset; /*make sure the velocity is set to offset value */
            }

            double N = (target_dist / (0.02 * ref_vel/2.24)); // speed is divided by 2.24 to convert into mps

            double x_point = x_add_on + (target_x)/N ;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

          //Rotate back the points to normal co-ordinates as we did transformation earlier */

          x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
          y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);

          }


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}