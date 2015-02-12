#include "ExplicitEuler.h"

void ExplicitEuler::integrate(SystemInterface* system, double timestep){
	system->setState(system->getState() + timestep * system->derivEval());
}

Eigen::VectorXd ExplicitEuler::integrate(Eigen::VectorXd state, Eigen::VectorXd derivEval, double timestep) {
	return (state + timestep * derivEval);
}