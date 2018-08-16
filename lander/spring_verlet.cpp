#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

  // declare variables
  double m, k, x, v, t_max, dt, t, a;
  vector<double> t_list, x_list, v_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  // simulation time and timestep
  t_max = 100;
  dt = 0.1;
  t_list.push_back(0); // FIND A BETTER WAY TO DO THIS
  t_list.push_back(dt);

  // load initial conditions into lists
  x_list.push_back(x);
  v_list.push_back(v);

  // use Euler to get second value for x
  x_list.push_back(x_list[0] + dt * v_list[0]);

  // Verlet integration
  for (t = 2 * dt; t <= t_max; t = t + dt) {

    // update time variable
    t_list.push_back(t);

    // calculate new position and velocity
	a = -k * x_list.back() / m;
	x = 2 * x_list.back() - x_list[x_list.size() - 2] + pow(dt, 2) * a;
	x_list.push_back(x);

	v = (x_list.back() - x_list[x_list.size() - 3]) / (2 * dt);
	v_list.push_back(v);

  }

  // append velocity array with final velocity
  v = (x_list.back() - x_list[x_list.size() - 2]) / dt;
  v_list.push_back(v);

  // Write the trajectories to file
  ofstream fout;
  fout.open("trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 1; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }
}
