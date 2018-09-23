#ifndef BBPP_H
#define BBPP_H

#include <vector>
#include <string>

using namespace std;

// planner class
class bbPP {
 private:

 	// global map vectors
 	vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // JMT trajectory producer
    vector<double> JMT(double T, vector<double> goal);

    // variables for constraints on velocity, acceleration, jerk
    double max_v;
    double max_a;
    double max_j;
    
    // car state variables for d, s, v and lane
    double current_d;
    double current_s;
    double current_v;
    int current_lane;

    // car target state variables for speed and lane
    double tgt_v;
    int target_lane;

    // environment state vector
	vector<double> environment_state;    

	// states for fsm simplified
	bool keep_lane;
	bool left_lane;
	bool right_lane;

	// for spline production, car x,y,yaw coordinates at each time step
	double car_x;
	double car_y;
	double car_yaw;

	// variable to control whether to use JMT or spline
	bool use_JMT;
 
 public:

  bbPP();

  virtual ~bbPP();

  // initialisation of environment
  void init(string map_file_);

  // state methods to: set car state, reset environment state, detect environment state
  void set_new_state(double p_size, double s, double d, double v, double end_s, double end_d, double x, double y, double yaw);
  void reset_environment_state();
  void detect_environment_state(vector< vector<double> > detected_cars);
  
  // helpers to: detect current lane, detect lane operation options, the best option, the initial speed
  int detect_current_lane(double d);
  void detect_options();
  void get_best_option();
  void set_initial_speed();

  // get the next move for the vehicle
  void getPath(vector<double>& next_x_vals,vector<double>& next_y_vals);

};


#endif /* BBPP_H */