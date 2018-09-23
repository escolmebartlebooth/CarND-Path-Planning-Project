#include "bbPP.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include "helper_functions.h"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

//
// Path Planner class definition implementation.
//
bbPP::bbPP() {}
bbPP::~bbPP() {}

void bbPP::init(string map_file_) {

  // initialise variables
  this->max_v = 49.50/2.24;
  this->max_a = 5;
  this->max_j = 10;
  
  this->current_s = 0;
  this->current_d = 0;
  this->current_v = 0;
  this->current_lane = 1;
  
  this->tgt_v = 0;
  this->target_lane = 1;
  
  this->keep_lane = true;
  this->left_lane = false;
  this->right_lane = false;
  
  this->reset_environment_state();
  
  // controls whether to use the JMT or Spline approach
  this->use_JMT = false;

  if (this->use_JMT) {
  	std::cout << "USING JMT: " << std::endl; 
  } else {
  	std::cout << "USING SPLINE: " << std::endl;
  }
  
  // load map
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  	this->map_waypoints_x.push_back(x);
  	this->map_waypoints_y.push_back(y);
  	this->map_waypoints_s.push_back(s);
  	this->map_waypoints_dx.push_back(d_x);
  	this->map_waypoints_dy.push_back(d_y);
  }

  std::cout << "path planner initialised" << std::endl;

}


// set the new state of the car on each iteration
void bbPP::set_new_state(double p_size, double s, double d, double v, double end_s, double end_d, double x, double y, double yaw) {
	// set the car's S and D values to the end previous path values if the path exists
	if (p_size > 0) {
		current_s = end_s;
		current_d = end_d;
	} else {
		current_s = s;
		current_d = d;
	}

	// update the car's x,y,yaw value
	this->car_x = x;
	this->car_y = y;
	this->car_yaw = yaw;

	// assume speed passed in mph, convert to m/s
	current_v = v / 2.24;

	// detect current lane
	this->current_lane = detect_current_lane(this->current_d);
}


// set the left, center, right lane state to initial values
void bbPP::reset_environment_state() {
	// each state pair is 0 or 1 to signify open / closed lane + max speed of lane
	this->environment_state = {0,this->max_v,0,this->max_v,0,this->max_v};
}


// taking the sensor fusion data, determine each lane's state
void bbPP::detect_environment_state(vector< vector<double> > detected_cars) {
	this->reset_environment_state();

	// loop though each car to work out whether impinging our car's options
	// current s will be the end of the path if there was a previous path
	for (int i = 0; i < detected_cars.size(); ++i)
	{
	  if (detected_cars[i][3] == this->current_lane) {
	  	// car is in same lane
	  	if ((detected_cars[i][0] > this->current_s) && (detected_cars[i][0] - this->current_s < 30)) {
	  		// car ahead of ego end path position but will be within 30m of the end path after the path has completed
	  		// update environment to 1 for CLOSED and make lane speed detected car's speed
	  		this->environment_state[2] = 1;
	  		this->environment_state[3] = detected_cars[i][2];
	  	}
	  } else if (detected_cars[i][3] - this->current_lane == -1) {
	  	// car is to left of us - for this one need to check absolute distance...
	  	if ((detected_cars[i][0] > this->current_s - 20) && (fabs(detected_cars[i][0] - this->current_s < 20))) {
	  		// car is ahead of current position but within 20m 
	  		// update environment to 1 for CLOSED and make lane speed detected car's speed
	  		this->environment_state[0] = 1;
	  		this->environment_state[1] = detected_cars[i][2];
	  	}
	  } else if (detected_cars[i][3] - this->current_lane == 1) {
	  	// car is to right of us
	  	if ((detected_cars[i][0] > this->current_s - 20) && (fabs(detected_cars[i][0] - this->current_s < 20))) {
	  		// car is ahead of current position but within 20m 
	  		// update environment to 1 for CLOSED and make lane speed detected car's speed
	  		this->environment_state[4] = 1;
	  		this->environment_state[5] = detected_cars[i][2];
	  	}
	  }
	}

	// set impossible states if our car is in one of the outer lanes...
	if (this->current_lane == 0) {
		// we can't go left
		this->environment_state[0] = -1;
	  	this->environment_state[1] = 0;
	}
	if (this->current_lane == 2) {
		// we can't go right
		this->environment_state[4] = -1;
	  	this->environment_state[5] = 0;
	}

	// write out the OPEN/CLOSED state of the environment
	std::cout << this->environment_state[0] << " " << this->environment_state[2] << " ";
	std::cout << this->environment_state[4] << " ";

	// detect what options are open to us
	this->detect_options();
}


// helper to determine the current lane of the car
int bbPP::detect_current_lane(double d) {
	// assumes 0-3.99 = 0; 4 to 7.99 = 1; 8 to 12 = 2; else -1
	if ((d>0) && (d<=3.99)) {
		return 0;
	} else if ((d>=4) && (d<=7.99)) {
		return 1;
	} else if ((d>=8) && (d<=12)) {
		return 2;
	} else {
		return -1;
	}
}


// based on environment state determine what we can do
void bbPP::detect_options() {
	// this shouldn't be needed as we have this embedded in environment state...
	if (this->environment_state[0] == 0) {
		this->left_lane = true;
	} else {
		this->left_lane = false;
	}
	// we can always stay in lane
	if (this->environment_state[2] == 0) {
		this->keep_lane = true;
	} else {
		this->keep_lane = true;
	}
	if (this->environment_state[4] == 0) {
		this->right_lane = true;
	} else {
		this->right_lane = false;
	}
}


// JMT trajectory producer
vector<double> bbPP::JMT(double T, vector<double> goal) {
	// this will create and return quintic polynomial

	// extract start and end points
	double si0 = goal[0];
	double si1 = goal[1];
	double si2 = goal[2];
	double sf0 = goal[3];
	double sf1 = goal[4];
	double sf2 = goal[5];

	// create A matrix
	MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
		 3*T*T, 4*T*T*T,5*T*T*T*T,
	     6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);	    
	B << sf0-(si0+si1*T+.5*si2*T*T),
	     sf1-(si1+si2*T),
	     sf2-si2;

	// get inverse matrix of A
	MatrixXd Ai = A.inverse();
	
	// Mat Mul to get answer
	MatrixXd C = Ai*B;

	// convert to vector and return coeffs
	vector <double> result = {si0, si1, .5*si2};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

	return result;
}


// based on environment and some simple rules, update the target lane and speed
void bbPP::get_best_option() {
	// the car is always in environment state position 2.
    int state_idx = 2;

    // get state of the lane the car is in 0 = clear; 1 = car ahead
    double I = this->environment_state[state_idx];

    // implement basic FSM here
    if (I == 0){
    	// KEEP LANE AND INCREASE SPEED TO MAX
    	std::cout << "KEEP LANE: "; 
    	this->tgt_v = min(this->tgt_v+0.224, max_v);
    } else {
    	// can we change left and is the speed higher
    	std::cout << "CONSIDER LANE CHANGE ";
    	std::cout << this->left_lane << " ";
    	std::cout << this->right_lane << " ";
    	std::cout << this->current_lane << " ";
    	if ((this->left_lane) && (this->current_lane != 0)) {
    		// Left lane open and we're not in lane zero...
    		std::cout << "CHANGE LEFT: "; 
    		this->target_lane = this->current_lane - 1;
    	} else if ((this->right_lane) && (this->current_lane != 2)) {
    		// right lane open and we're not in lane two
    		std::cout << "CHANGE RIGHT: "; 
    		this->target_lane = this->current_lane + 1;
    	} else {
    	  std::cout << "KEEP LANE IS BETTER: ";
    	  this->tgt_v = max(this->tgt_v-0.224, this->environment_state[3]); 	
    	} 
    }
    std::cout << this->target_lane << std::endl;
}


// set the speed of the ego to current v if we're up and running (used in JMT only)
void bbPP::set_initial_speed() {
	if (this->current_v != 0) {
    	this->current_v = this->tgt_v;
    }
}


// path planner
void bbPP::getPath(vector<double>& next_x_vals,vector<double>& next_y_vals) {
    // time period for the path
    double T = 2;

    // set the initial speed to the tgt_v if we have a path
    this->set_initial_speed();

    // set target lane and speed based on environment
    this->get_best_option();  

    
    // switch between JMT and SPLINE
    if (this->use_JMT) {
    	// get s coeffs
	    double dist = this->current_v*T + 0.5*((this->tgt_v - this->current_v)/T)*T*T;
	    vector<double> goal = {this->current_s, this->current_v, 0, this->current_s+dist, this->tgt_v, 0};
	    vector <double> s_coeffs = this->JMT(T, goal);
	    
	    // get d coeffs
	    double tgt_d = this->target_lane*4 + 2;
	    vector<double> d_goal = {this->current_d, 0, 0, tgt_d, 0, 0};
	    vector <double> d_coeffs = this->JMT(T, d_goal);
	    
	    // generate trajectory but only add a few more steps...
	    int len_new_path = 50 - next_x_vals.size();
	    for (int i = 0; i < len_new_path; ++i) {
	    	double t = 0.02*(i+1);
	        double next_s = s_coeffs[0] + s_coeffs[1]*t + s_coeffs[2]*t*t + s_coeffs[3]*t*t*t + s_coeffs[4]*t*t*t*t + s_coeffs[5]*t*t*t*t*t;
	        double next_d = d_coeffs[0] + d_coeffs[1]*t + d_coeffs[2]*t*t + d_coeffs[3]*t*t*t + d_coeffs[4]*t*t*t*t + d_coeffs[5]*t*t*t*t*t;
	        vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	        next_x_vals.push_back(xy[0]);
	        next_y_vals.push_back(xy[1]);
	    }
	    // probably have to smooth based on Eigen and then regenate y based on x...
	    const size_t n_points = next_x_vals.size();
	    auto car_ptsx = Eigen::VectorXd(n_points);
	    auto car_ptsy = Eigen::VectorXd(n_points);

	    for (unsigned int i=0; i < n_points; i++){
	        car_ptsx(i) = next_x_vals[i];
	        car_ptsy(i) = next_y_vals[i];
	    }

	    auto coeffs = polyfit(car_ptsx, car_ptsy, 3);

	    for (unsigned int i=0; i < n_points; i++){
	      next_y_vals[i] = polyeval(coeffs, next_x_vals[i]);
	    }

	} else {
	    	// create placeholder for spline points
	        vector<double> ptsx;
		    vector<double> ptsy;

		    // set ref x,y, yaw to current car position
		    double ref_x = this->car_x;
		    double ref_y = this->car_y;
		    double ref_yaw = deg2rad(this->car_yaw);

		    // find out how many previous path points there are
		    int len_old_path = next_x_vals.size();

		    // add a couple of points to the point vectors
		    if (len_old_path < 2) {
		    	// if there's not much previous path use the current car coords
		    	double prev_car_x = car_x - cos(deg2rad(car_yaw));
		    	double prev_car_y = car_y - sin(deg2rad(car_yaw));

		    	ptsx.push_back(prev_car_x);
		    	ptsx.push_back(car_x);

		    	ptsy.push_back(prev_car_y);
		    	ptsy.push_back(car_y);

	    } else {
	    	// if there's some path left over, use the last 2 points of that path
	    	ref_x = next_x_vals[len_old_path-1];
	    	ref_y = next_y_vals[len_old_path-1];

	    	double ref_x_prev = next_x_vals[len_old_path-2];
	    	double ref_y_prev = next_y_vals[len_old_path-2];
	    	ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

	    	ptsx.push_back(ref_x_prev);
	    	ptsx.push_back(ref_x);

	    	ptsy.push_back(ref_y_prev);
	    	ptsy.push_back(ref_y);
	    }

	    // create 3 sparsely spaced waypoints at 30, 60, 90 points from the current s value and target lane
	    vector<double> next_wp0 = getXY(this->current_s+30,(2+4*this->target_lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    vector<double> next_wp1 = getXY(this->current_s+60,(2+4*this->target_lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    vector<double> next_wp2 = getXY(this->current_s+90,(2+4*this->target_lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);


	    // put the points onto the vectors
	    ptsx.push_back(next_wp0[0]);
	    ptsx.push_back(next_wp1[0]);
	    ptsx.push_back(next_wp2[0]);

	    ptsy.push_back(next_wp0[1]);
	    ptsy.push_back(next_wp1[1]);
	    ptsy.push_back(next_wp2[1]);

	    // convert the points to car centric coordinates
	    for (int i = 0; i < ptsx.size(); ++i)
	    {
	      double shift_x = ptsx[i] - ref_x;
	      double shift_y = ptsy[i] - ref_y;

	      ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
	      ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
	    }

	    // the magic spline part, initialise and create a spline function from the points
	    tk::spline spl;

	    spl.set_points(ptsx,ptsy);

	    // we want to move 30 metres forward
	    double target_x = 30.0;
	    double target_y = spl(target_x);
	    double target_distance = sqrt(target_x*target_x+target_y*target_y);

	    double x_add_on = 0;

	    // add more points to end of last path to get to 50 total points...
	    for (int i = 0; i <= 50-len_old_path; ++i)
	    {
	    	double N = (target_distance/(0.02*tgt_v));
	    	double x_point = x_add_on+target_x/N;
	    	double y_point = spl(x_point);

	    	x_add_on = x_point;

	    	double x_ref = x_point;
	    	double y_ref = y_point;

	    	// convert back to global map points
	    	x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
	    	y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

	    	x_point += ref_x;
	    	y_point += ref_y;

	    	next_x_vals.push_back(x_point);
	    	next_y_vals.push_back(y_point);
	    }	
    }
    
}