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

bool evolution_sort(vector<double> i, vector<double> j) { return i[3] > j[3]; }

void evolution(int mode)
// This function handles population generation, reproduction and mutation 
// for the autopilot's evolutionary algorithm
{
	// Initialising local variables
	static vector<double> specimen, best_specimen(4);
	static vector<vector<double>> population, new_population;
	double Kp_rand, Kh_rand, delta_rand;
	static int iter_count = 0;
	static unsigned short prev_scenario = scenario;
	int n, sum_n, cycles;

	n = 10;
	sum_n = n * (n + 1) / 2;
	cycles = 20;

	// Resetting conditions upon scenario change
	if (prev_scenario != scenario) {
		prev_scenario = scenario;
		population.clear();
		new_population.clear();
		iter_count = 0;
	}

	if (mode == 0) {
		// Population restructuring and specimen selection
	
		// Incrementing iteration counter
		iter_count++;
		//cout << iter_count << endl;

		// Setting up initial population
		if (iter_count == 1) {
			cout << "Begin training" << endl;
			cout << "Cycle 1 of " + to_string(cycles) << endl;
			// GENERATE RANDOM PARAMETERS - may not need individual variables
			for (int i = 0; i < sum_n; i = i + 1) {
				specimen.clear();
				Kh_rand = (double)(rand() % 1000) / 1000;
				Kp_rand = (double)(rand() % 1000) / 1000;
				delta_rand = (double)(rand() % 1000) / 1000;
				specimen.push_back(Kh_rand);
				specimen.push_back(Kp_rand);
				specimen.push_back(delta_rand);
				population.push_back(specimen);
			}
			specimen = population[0];
		}
		else if (iter_count % sum_n == 1) {
			// Reproduction and mutation
			// Sort by fitness
			// Take top 14
			// Breed each specimen with every other specimen and randomly mutate
			cout << "Cycle " + to_string((int)((iter_count - 1) / sum_n + 1)) + " of " + to_string(cycles) << endl;
			sort(new_population.begin(), new_population.end(), evolution_sort);
			new_population.resize(n);
			if (new_population[0][3] > best_specimen[3]) { best_specimen = new_population[0]; }
			population = new_population;
			for (int i = 0; i < n; i = i + 1) {
				for (int j = 1 + i; j < n; j = j + 1) {
					specimen.clear();
					specimen.push_back((new_population[i][0] + new_population[j][0]) / 2 + (double)(rand() % 50) / 1000 - 0.025);
					specimen.push_back((new_population[i][1] + new_population[j][1]) / 2 + (double)(rand() % 50) / 1000 - 0.025);
					specimen.push_back((new_population[i][2] + new_population[j][2]) / 2 + (double)(rand() % 50) / 1000 - 0.025);
					population.push_back(specimen);
				}
			}
			specimen = population[0];
			//cout << best_specimen[0] << ' ' << best_specimen[1] << ' ' << best_specimen[2] << ' ' << best_specimen[3] << endl;
		}
		else {
			specimen = population[(iter_count - 1) % sum_n];
		}

		// Updating test parameters
		EA_Kh = specimen[0];
		EA_Kp = specimen[1];
		EA_delta = specimen[2];

		if (iter_count == cycles * sum_n) {
			// Write the parameters to file
			ofstream fout;
			fout.open("autopilot_parameters/case_" + to_string(scenario) + ".txt");
			if (fout) { // file opened successfully
				fout << best_specimen[0] << ' ' << best_specimen[1] << ' ' << best_specimen[2] << ' ' << best_specimen[3] << endl;
				fout.close();
			}
			else { // file did not open successfully
				cout << "Could not open file for writing" << endl;
			}
			cout << "Training complete" << endl;
			while (true) {}
		}
	}
	else if (mode == 1) {
		// Fitness mode
		if (crashed == true) { specimen.push_back(0.0); }
		else { specimen.push_back(fuel); //cout << specimen[0] << ' ' << specimen[1] << ' ' << specimen[2] << ' ' << specimen[3] << endl;
		}
		new_population.push_back(specimen);
	}
}

void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
	// Initialising local variables
	double Kh, Kp, delta, h, error_t, error_r, P, target_altitude, calculated_entry_angle, entry_angle = 30; // may need static
	bool safe_for_parachute = safe_to_deploy_parachute();
	static bool fileopen, disable_engine = false;
	pair <int, int> target_angle;
	vector3d target_velocity, transverse_direction;

	// Defining relevant variables
	h = position.abs() - MARS_RADIUS;
	target_altitude = 300000;
	transverse_direction = ((position ^ velocity) ^ position).norm();

	if (autopilot_mode == REENTRY) {
		stabilized_attitude_angle = make_pair(225, 0);
		// Deactivate thrusters when angle of entry is below the exosphere
		calculated_entry_angle = ((transverse_direction * (MARS_RADIUS + EXOSPHERE) - position).norm() ^ velocity.norm()).abs();
		if (calculated_entry_angle > 0.5 && !disable_engine) { throttle = 0.1; }
		else { disable_engine = true;  throttle = 0.0; }
		if (h < EXOSPHERE) { disable_engine = false; autopilot_mode = DESCENT; }
	}

	else if (autopilot_mode == DESCENT || autopilot_mode == E_DESCENT) {
		if (autopilot_mode == DESCENT) { // Predefined values
			// Read in parameters from relevant file
			ifstream file("autopilot_parameters/case_" + to_string(scenario) + ".txt");
			if (file) { // file opened successfully
				file >> Kh >> Kp >> delta;
			}
			else { // file did not open successfully
				cout << "Could not open file for reading" << endl;
			}
		}

		else { // Evolutionary algorithm
			Kh = EA_Kh;
			Kp = EA_Kp;
			delta = EA_delta;
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
	else if (autopilot_mode == INJECTION) { 
		// Defining variables
		target_angle = make_pair(90, 0);
		delta = grav.abs() / MAX_THRUST;

		// Determining required thrust
		Kp = 1e-4;
		target_velocity = sqrt(GRAVITY * MARS_MASS / (target_altitude + MARS_RADIUS)) * transverse_direction;
		error_t = target_velocity.abs2() - target_velocity * velocity; // transverse error
		error_r = (target_altitude - h) - position.norm() * velocity * 1000; // radial error

		// Converting errors into throttle and attitude stabilization commands
		P = error_r * Kp; // scaling for correct throttle control
		if (P > 1 && error_t >= 0) { throttle = 1; stabilized_attitude_angle.first = entry_angle; }
		else if (P > 1.0 && error_t < 0.0) { throttle = 1; stabilized_attitude_angle.first = 360 - entry_angle; }
		else if (P > delta * -1.0 && P < 1.0 - delta && error_t > 0.0) { throttle = P + delta; stabilized_attitude_angle.first = entry_angle; }
		else if (P > delta * -1.0 && P < 1.0 - delta && error_t < 0.0) { throttle = P + delta; stabilized_attitude_angle.first = 360 - entry_angle; }
		else if (P >= -1.0 && P < delta * -1.0 && error_t > 0.0) { throttle = P * -1.0 - delta; stabilized_attitude_angle.first = 180 - entry_angle; }
		else if (P >= -1.0 && P < delta * -1.0 && error_t < 0.0) { throttle = P * -1.0 - delta; stabilized_attitude_angle.first = 180 + entry_angle; }
		else if (P < -1.0 && error_t > 0.0) { throttle = 1.0 - delta; stabilized_attitude_angle.first = 180 - entry_angle; }
		else if (P < -1.0 && error_t < 0.0) { throttle = 1.0 - delta; stabilized_attitude_angle.first = 180 + entry_angle; }

		// Deactivating autopilot - determine best conditions for this
		if (abs(position.norm() * velocity) < 0.32 && abs(target_altitude - h) < 10000) { 
		throttle = 0;
		autopilot_mode = REENTRY; 
		autopilot_enabled = false; }
	}
}

void numerical_dynamics(void)
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

void initialize_simulation(void)
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
		autopilot_enabled = true;
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
		autopilot_enabled = true;
		autopilot_mode = REENTRY;
		break;

	case 3:
		// polar surface launch at escape velocity (but drag prevents escape)
		position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
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
		autopilot_enabled = true;
		autopilot_mode = DESCENT;
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
		autopilot_mode = E_DESCENT;
		break;

	case 6:
		// an aerostationary stationary orbit
		position = vector3d(0.0, -aero_altitude, 0.0);
		velocity = vector3d(aero_velocity, 0.0, 0.0);
		orientation = vector3d(0.0, 0.0, 90.0);
		delta_t = 0.1;
		parachute_status = NOT_DEPLOYED;
		stabilized_attitude = true;
		autopilot_enabled = true;
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
	if (autopilot_mode == E_DESCENT || autopilot_mode == REENTRY) { evolution(0); } // Initialises evolutionary algorithm
}
