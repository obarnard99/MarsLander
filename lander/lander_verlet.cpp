// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    // Initialising local variables
	double Kh, Kp, delta, h, error_t, error_r, P, target_altitude; // may need static
	bool safe_for_parachute = safe_to_deploy_parachute();
	static bool fileopen, disable_engine = false;
	pair <int, int> target_angle;
	vector3d target_velocity, transverse_direction;

	// Defining relevant variables
	h = position.abs() - MARS_RADIUS;
	target_altitude = 300000; 

	if (autopilot_mode == REENTRY) {
		stabilized_attitude_angle = make_pair(225, 0);
		throttle = 0.1;
		if (h < EXOSPHERE) { autopilot_mode = DESCENT; }
	}

	else if (autopilot_mode == DESCENT || autopilot_mode == E_DESCENT) {
		// Defining relevant variables
		if (autopilot_mode == DESCENT) { // Predefined values
			Kh = 0.025; // 0.025
			Kp = 0.6; // 0.6
			delta = grav.abs() / MAX_THRUST; // 0.4
		}

		else { // Evolutionary algorithm
			
			// Read from EA file
			static fstream fout;
			if (not fileopen) {
				fout.open("EA.txt");
				fileopen = true;
			}

			Kh = 0.025; // 0.025
			Kp = 0.6; // 0.6
			delta = grav.abs() / MAX_THRUST; // 0.4

			if (fout) { // file opened successfully
				fout << Kh << ' ' << Kp << ' ' << delta << endl;
			}
			else { // file did not open successfully
				cout << "Could not open EA file for writing" << endl;
			}
		}

		// Reorienting
		stabilized_attitude_angle = make_pair(0, 0);

		// Determining required thrust
		error_r = -(0.5 + Kh * h + velocity * position.norm());
		P = Kp * error_r;

		if (P <= delta * -1) { throttle = 0; }
		else if (P > delta * -1 && P < 1 - delta) { throttle = P + delta; }
		else { throttle = 1; }

		// Deploying parachute
		if (safe_for_parachute && position.abs() - MARS_RADIUS < 100000)
		{
			parachute_status = DEPLOYED;
		}
	}
	else if (autopilot_mode == INJECTION) { // Needs finishing
		// Defining variables
		target_angle = make_pair(90, 0);
		delta = grav.abs() / MAX_THRUST;

		// Determining required thrust
		
		// v1
		// Kp = 0.0001;
		// transverse_speed = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2));
		// target_speed = sqrt(GRAVITY * MARS_MASS / (target_altitude + MARS_RADIUS));
		// if (h > target_altitude || disable_engine == true) { throttle = 0; disable_engine = true; }
		// else {
		//	error = target_speed - transverse_speed; // very bad algorithm
		//	P = abs(Kp * error);

		//	if ((vector3d)(position.norm() * velocity).abs2() < 10000 || P > 1) { throttle = 1; }
		//	else if (P <= 0.005) { throttle = 0; }
		//	else if (P < 1) { throttle = P; } }
		
		// v2
		Kp = 1e-4;
		transverse_direction = ((position ^ velocity) ^ position).norm();
		target_velocity = sqrt(GRAVITY * MARS_MASS / (target_altitude + MARS_RADIUS)) * transverse_direction;
		error_t = target_velocity.abs2() - target_velocity * velocity;
		P = error_t * Kp;

		//if ( h < EXOSPHERE * 0.8) { throttle = 1; stabilized_attitude_angle.first = 30; }
		//else if (P <= 0.005) { throttle = 0; stabilized_attitude_angle.first = 0; }
		//else if (P <= 1) { throttle = P; stabilized_attitude_angle.first = 90; }
		//else { throttle = 1; stabilized_attitude_angle.first = 90; }
		
		error_r = (target_altitude - h) - position.norm() * velocity * 100; 
		P = error_r * Kp;

		// if ( h < EXOSPHERE * 0.5) { throttle = 1; stabilized_attitude_angle.first = 0; }
		//else if (position.norm() * velocity > 500) { throttle = 0; }
		if (P > 1) { throttle = 1; stabilized_attitude_angle.first = 30; }
		else if (P > delta * -1 && P < 1 - delta) { throttle = P + delta; stabilized_attitude_angle.first = 30; }
		else if (P >= -1 && P < delta * -1) { throttle = P * -1 - delta; stabilized_attitude_angle.first = 150; }
		else if (P < -1) { throttle = 1 - delta; stabilized_attitude_angle.first = 150; }

		// cout <<  << endl;

		// Determining required angle
		// v1
		// stabilized_attitude_angle.first = (0.000005 * (h - target_altitude) / (1 + 0.000005 * abs(h - target_altitude))) * target_angle.first + target_angle.first;
		// v2
		// stabilized_attitude_angle.first = (h / (h + target_altitude)) * (target_angle.first - 15) * 2 + 30;
		// v3
		// stabilized_attitude_angle.first = 

		// Deactivating autopilot - determine best conditions for this
		//if (abs(position.norm() * velocity) < 0.32 && abs(target_altitude - h) < 10000) { 
			//throttle = 0;
			//autopilot_mode = REENTRY; 
			//autopilot_enabled = false; }
	}
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
    // Initialising local variables
	double rho, lander_area, chute_area, mass;
	static bool parity;
	vector3d drag, acceleration; // May need to make these global variables
	static vector3d previous_position_odd, previous_position_even;

	// Defining relevant variables
	rho = atmospheric_density(position);
	mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;
	lander_area = pow(LANDER_SIZE, 2) * M_PI; // Does this change??????
	chute_area = 5 * pow(2 * LANDER_SIZE, 2);

	// Calculating forces
	thrust = thrust_wrt_world();
	grav = -((GRAVITY * MARS_MASS * mass) / position.abs2()) * position.norm();

	if (parachute_status == DEPLOYED) {
		drag = -0.5 * rho * (DRAG_COEF_LANDER * lander_area + DRAG_COEF_CHUTE * chute_area) * velocity.abs() * velocity;
	}
	else {
		drag = -0.5 * rho * DRAG_COEF_LANDER * lander_area * velocity.abs() * velocity;
	}
	acceleration = (thrust + drag + grav) / mass;

	// Use Euler to find initial displacement
	if (simulation_time == 0.0) {
		previous_position_even = position;
		position = position + delta_t * velocity;
		parity = false;
	}
	else if (parity) {
		previous_position_even = position;

		// Calculating new position and velocity
		position = 2 * position - previous_position_odd + pow(delta_t, 2) * acceleration;
		velocity = (position - previous_position_odd) / (2 * delta_t);

		// Redefining parity
		parity = false;
	}
	else {
		previous_position_odd = position;

		// Calculating new position and velocity
		position = 2 * position - previous_position_even + pow(delta_t, 2) * acceleration;
		velocity = (position - previous_position_even) / (2 * delta_t);

		// Redefining parity
		parity = true;
	}

	// Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
	if (stabilized_attitude) attitude_stabilization();

    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
	if (autopilot_enabled) {
		stabilized_attitude = true;
		autopilot();
	}
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  // Initialising relevant variables
  static double aero_altitude, aero_velocity;
  aero_altitude = cbrt((GRAVITY * MARS_MASS * pow(MARS_DAY, 2)) / (4 * pow(M_PI, 2)));
  aero_velocity = sqrt((GRAVITY * MARS_MASS) / aero_altitude);

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "aerostationary orbit";
  scenario_description[7] = "orbital injection from 10km";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	autopilot_mode = REENTRY;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
	autopilot_mode = DESCENT;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	autopilot_mode = REENTRY;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
	autopilot_mode = INJECTION;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
	autopilot_mode = REENTRY;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = true;
	autopilot_mode = DESCENT;
    break;

  case 6:
	// an aerostationary stationary orbit
	position = vector3d(0.0, -aero_altitude, 0.0);
    velocity = vector3d(aero_velocity, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
	autopilot_mode = REENTRY;
    break;

  case 7:
	  // an orbital injection from rest at 10km altitude
	  position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
	  velocity = vector3d(0.0, 0.0, 0.0);
	  orientation = vector3d(0.0, 0.0, 90.0);
	  delta_t = 0.1;
	  parachute_status = NOT_DEPLOYED;
	  stabilized_attitude = true;
	  autopilot_enabled = true;
	  autopilot_mode = INJECTION;
	  break;

  case 8:
    break;

  case 9:
    break;

  }
}
