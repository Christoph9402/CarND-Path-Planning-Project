
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
using std::string;
using std::vector;

int main() {
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

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
    int lane = 1;
    //double lane_width = 4.0;
    double max_speed = 0.0;


  h.onMessage([&max_speed, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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



            /**
             * TODO: define a path made up of (x,y) points that the car will visit
             *   sequentially every .02 seconds
             */
            int previous_size = previous_path_x.size();


            if (previous_size > 0) {
                car_s = end_path_s;
            }

            bool TooClose = false;

            double speed_car_ahead = 0.0;
            //Create 3 vectors that will store the indices of the vehicles nearby in every lane
            vector<double> VehiclesLane0;
            vector<double> VehiclesLane1;
            vector<double> VehiclesLane2;
            //Loop through the sensor fusion list and extract d
            for (int i = 0; i < sensor_fusion.size(); i++) {
                float d = sensor_fusion[i][6];
                //Check, if car is in the same lane as ego vehicle
                if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
                    //speed and s value of the car in front
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);
                    double check_car_s = sensor_fusion[i][5];
                    speed_car_ahead = check_speed;
                    check_car_s += ((double) previous_size * 0.02 * check_speed);

                    if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                        //Flag as too close and print in terminal
                        TooClose = true;
                        std::cout<<"Vehicle in front is too close!\n";
                    }
                }
            }

            //Check every vehicle in the sensor fusion list and extract the vehicles d value
            for (int i = 0; i < sensor_fusion.size(); i++) {
                float d = sensor_fusion[i][6];
                //Check, if car is in lane 0
                if (d <= (4) && d > (0)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);
                    double check_car_s = sensor_fusion[i][5];
                    speed_car_ahead = check_speed;
                    check_car_s += ((double) previous_size * 0.02 * check_speed);
                    //Check, if the vehicle in lane 0 is within a distance of 40m in front and 20m behind
                    if (((check_car_s > car_s) && (abs(check_car_s-car_s) < 40))||((check_car_s <= car_s) && abs(check_car_s-car_s) <10)) {
                        //Append vehicle to the vehicleslane0 list
                        VehiclesLane0.push_back(i);
                        //std::cout<<"Lane 0 NOT empty\n";
                    }
                //Check if the car is in lane 1
                } else if (d <= (8) && d > (4)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);
                    double check_car_s = sensor_fusion[i][5];
                    speed_car_ahead = check_speed;
                    check_car_s += ((double) previous_size * 0.02 * check_speed);
                    //Check, if the vehicle in lane 1 is within a distance of 40m in front and 20m behind
                    if (((check_car_s > car_s) && (abs(check_car_s-car_s) < 40))||((check_car_s <= car_s) && abs(check_car_s-car_s) < 10)) {
                        //Append vehicle to the vehicleslane1 list
                        VehiclesLane1.push_back(i);
                        //std::cout<<"Lane 1 NOT empty\n";
                    }
                //Check if the car is in lane 2
                } else if (d <= (12) && d > (8)) {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx * vx + vy * vy);
                    double check_car_s = sensor_fusion[i][5];
                    speed_car_ahead = check_speed;
                    check_car_s += ((double) previous_size * 0.02 * check_speed);
                    //Check, if the vehicle in lane 2 is within a distance of 40m in front and 20m behind
                    if (((check_car_s > car_s) && (abs(check_car_s-car_s) < 40))||((check_car_s <= car_s) && abs(check_car_s-car_s) < 10)) {
                        //Append vehicle to the vehicleslane2 list
                        VehiclesLane2.push_back(i);
                        //std::cout<<"Lane 2 NOT empty\n";
                    }
                }
            }
            //Print number of vehicles in lanes
            std::cout << "lane: " << lane << "\t Vehicles lane0: " << VehiclesLane0.size() << "\t vehicles lane 1: " << VehiclesLane1.size() << "\t Vehicles lane 2: " << VehiclesLane2.size() << "\t too close: " << TooClose << std::endl;


            if (TooClose) {
                //define cases for lane changes

                //if ego vehicle is in lane 0 and the vehicleslane1 vector is empty, the ego vehicle can change lanes without problem. At the same time the ego vehicle can increase its speed, if it was lower before
                if((lane==0)&&(VehiclesLane1.empty())){
                    //increase lane by one --> change one lane to the right
                    lane++;
                    if (max_speed < 49.5) {
                        max_speed += 0.35;}
                }
                //if ego vehicle is in lane 0 and lane 1 is not empty, but lane 2 --> decrease speed to avoid max jerk and change lane
                else if ((lane==0)&&(!VehiclesLane1.empty())&&(VehiclesLane2.empty())){
                    max_speed-=0.35;
                    lane+2;
                }
                //If ego vehicle is in lane 1 and the lane to the left is empty, while lane to the right is not --> change lane to left and if possible start increasing speed
                else if((lane==1)&&(VehiclesLane0.empty())&&(!VehiclesLane2.empty())){
                    lane--;
                    if (max_speed < 49.5) {
                        max_speed += 0.35;}
                }
                //is ego vehicle is in lane 1 and the lane to the left is not empty, while lane to the right is --> change lane to right and if possible start increasing speed
                else if((lane==1)&&(VehiclesLane2.empty())&&(!VehiclesLane0.empty())){
                    lane++;
                    if (max_speed < 49.5) {
                        max_speed += 0.35;}
                }
                //If ego vehicle is in lane 1 and lane 0 and lane 2 are empty, change lane to left and if possible increase speed
                else if((lane==1)&&(VehiclesLane0.empty())&&(VehiclesLane2.empty())){
                    lane--;
                    if (max_speed < 49.5) {
                        max_speed += 0.35;}
                }
                //If ego vehicle is in lane 2 and lane 1 is empty, change lane to the left
                else if ((lane==2)&&(VehiclesLane1.empty())){
                    lane--;
                    if (max_speed < 49.5) {
                        max_speed += 0.35;}
                }
                //If ego vehicle is in lane 2 and lane 1 is not empty, but lane 0 is --> change lane to lane 0
                else if ((lane==2)&&(!VehiclesLane1.empty())&&(VehiclesLane0.empty())){
                    max_speed-=0.35;
                    lane-2;
                }
                else if((!VehiclesLane0.empty())&&(!VehiclesLane1.empty())&&(!VehiclesLane2.empty())){
                    //Check speed of car in front and reduce speed until same speed as car in front
                    if(max_speed >= speed_car_ahead){
                        max_speed -= 0.35;
                    }
                    //Try to maintain the speed of car in front
                    else if(max_speed < speed_car_ahead){
                        max_speed += 0.35;
                    }
                    //Check speed of cars on other lanes

                    //
                }

            }
            //If the vehicle in front is not too close to our vehicle and our car is to slow, increase speed until 49,5mph
            else if (max_speed < 49.5) {
                max_speed += 0.35;
            }


//Code implemented that was presented in FaQ:
//Create 2 vectors for points that make the path
          vector<double>points_path_x;
          vector<double>points_path_y;
          // Defining ref_x, ref_y as well as ref_yaw
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(previous_size<2){
              double previous_car_x = car_x - cos(car_yaw);
              double previous_car_y = car_y - sin(car_yaw);

              points_path_x.push_back(previous_car_x);
              points_path_x.push_back(car_x);
              points_path_y.push_back(previous_car_y);
              points_path_y.push_back(car_y);
          }
          else {
              ref_x=previous_path_x[previous_size-1];
              ref_y=previous_path_y[previous_size-1];
              double previous_ref_x = previous_path_x[previous_size-2];
              double previous_ref_y = previous_path_y[previous_size-2];
              ref_yaw=atan2(ref_y-previous_ref_y,ref_x-previous_ref_x);

              points_path_x.push_back(previous_ref_x);
              points_path_x.push_back(ref_x);
              points_path_y.push_back(previous_ref_y);
              points_path_y.push_back(ref_y);
          }
          vector<double> next_waypoint0=getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          points_path_x.push_back(next_waypoint0[0]);
          points_path_y.push_back(next_waypoint0[1]);

          vector<double> next_waypoint1=getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          points_path_x.push_back(next_waypoint1[0]);
          points_path_y.push_back(next_waypoint1[1]);

          vector<double> next_waypoint2=getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          points_path_x.push_back(next_waypoint2[0]);
          points_path_y.push_back(next_waypoint2[1]);


            for(int i=0;i<points_path_x.size();i++){
              double shift_x=points_path_x[i]-ref_x;
              double shift_y=points_path_y[i]-ref_y;
              points_path_x[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              points_path_y[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }
          tk::spline s;

          s.set_points(points_path_x,points_path_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0;i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          double target_x=30.0;
          double target_y=s(target_x);
          double target_distance=sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on=0;

          for(int i=1;i<=50-previous_path_x.size();i++){
              double N=(target_distance/(0.02*max_speed/2.24));
              double x_point=x_add_on+(target_x)/N;
              double y_point=s(x_point);
              x_add_on=x_point;
              double x_ref=x_point;
              double y_ref=y_point;

              //
              x_point=(x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              x_point +=ref_x;

              y_point=(x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
              y_point +=ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
            json msgJson;
            // Run Simulator
            // mkdir build && cd build
            // cmake .. && make
            // ./path_planning
            //mkdir build && cd build && cmake .. && make && ./path_planning


            //################################ End of path planning code section############################

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
    return  -1;
  }
  
  h.run();
}