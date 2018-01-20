#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
			std::cout << "[VEH] State: " << *it << "cost: " << cost << endl;
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
		if(lane != 0) {
			states.push_back("PLCL");
		}
		if (lane != lanes_available - 1) {
			states.push_back("PLCR");
		}
    } else if (state.compare("PLCL") == 0) {
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != lanes_available - 1) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->max_acceleration * 0.02 + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
			std::cout << "[VEH] VB: Vehicle behind use velocity: " << new_velocity << endl;
        } else {
            //float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v * 0.02 - 0.5 * (this->a) * 0.02 * 0.02; // Equation: d = d0+ vt -0.5a(t^2)

			//float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v * 0.02 - 0.5 * (this->a); // Equation: d = d0+ vt -0.5a(t^2)
			//float max_velocity_in_front = (vehicle_ahead.s + vehicle_ahead.v * 0.02 - this->s); // Equation: v = s1-s0/t
			float max_velocity_in_front = (vehicle_ahead.v); // Equation: v = s1-s0/t
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed); 
			//std::cout << "[VEH] NVB: Vehical ahead s: " << vehicle_ahead.s << endl;
			//std::cout << "[VEH] NVB: Vehical ahead velocity: " << vehicle_ahead.v << endl;
			//std::cout << "[VEH] NVB: New velocity: " << new_velocity << endl;
			//std::cout << "[VEH] NVB: New accel: " << new_accel << endl;
			//std::cout << "[VEH] NVB: Max velocity with accel: " << max_velocity_accel_limit << endl;
			//std::cout << "[VEH] NVB:Max velocity in front: " << max_velocity_in_front << endl;
        }
		
		new_accel = (new_velocity - this->v) / 0.02;
		if (abs(new_accel) > this->max_acceleration)
		{
			new_velocity = this->v - (this->max_acceleration * 0.02);
			std::cout << "[VEH] NVB: Smoothing velocity" << endl;

			if (new_velocity <= 0)
			{
				new_velocity = 0.1;
			}
		}

    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
		//std::cout << "[VEH] NVA: Max velocity with accel: " << max_velocity_accel_limit << endl;
    }
    
	//std::cout << "[VEH]: Final velocity: " << new_velocity << endl;
    new_accel = (new_velocity - this->v) / 0.02; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity * 0.02 + (new_accel * 0.02 * 0.02) / 2.0; // Equation distance = s + vt + 0.5a(t^2)
	//std::cout << "[VEH]: Final acceleration: " << new_accel << endl;
	//std::cout << "[VEH]: Final position: " << new_position << endl;
    return{new_position, new_velocity, new_accel};
    
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
		int max_gap = this->preferred_buffer;
		float dist_between_veh = abs((this->s + this->v * 0.02) - (next_lane_vehicle.s + next_lane_vehicle.v * 0.02));
        if (dist_between_veh <= max_gap && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
			std::cout << "[VEH] Vehicle found in next lane - aborting lane change" << endl;
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

float Vehicle::position_at(double t) {
	//cout << "[VEH] position_at() v: " << this->v << endl;
    return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_gap = this->preferred_buffer;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
		float dist_between_veh = abs(this->s - temp_vehicle.s);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && (dist_between_veh <= max_gap)) {
			std::cout << "[VEH] s of ego: "<< this->s << " s of veh behind: "<< temp_vehicle.s << " Vehicle behind of dist: " << dist_between_veh << endl;
			//max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = this->preferred_buffer * 2 + this->s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	vector<Vehicle> predictions;
	float delta_t;
    for(int i = 0; i < horizon; i++) {
	  delta_t = 0.02 * i;
      float next_s = position_at(i * delta_t);
	  //cout << "[VEH] Current s: " << s << endl;
	  //cout << "[VEH] Predicted next_s: " << next_s << endl;
      float next_v = 0;
      if (i < horizon-1) {
		delta_t = 0.02 * i + 1;
        next_v = position_at(delta_t) - s;
      }
	  //cout << "[VEH] Predicted v: " << next_v << endl;
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<float> road_data) {
	/*
	Called by simulator before simulation begins. Sets various
	parameters which will impact the ego vehicle.
	*/
	target_speed = road_data[0];
	lanes_available = (int)road_data[1];
	goal_s = (int)road_data[2];
	goal_lane = (int)road_data[3];
	max_acceleration = road_data[4];
}