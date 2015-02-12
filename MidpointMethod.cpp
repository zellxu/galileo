#include "MidpointMethod.h"

void MidpointMethod::integrate(SystemInterface* system, double timestep){
	Eigen::VectorXd initState = system->getState();
	system->setState(initState + timestep/2 * system->derivEval());
	system->setState(initState + timestep * system->derivEval());
}

Eigen::VectorXd MidpointMethod::integrate(Eigen::VectorXd state, Eigen::VectorXd derivEval, double timestep) {

	Eigen::VectorXd halfstep = state + timestep/2.0 * derivEval;

	//Manually evaluating because I won't do this on MyWorldClass. :(
	Eigen::VectorXd fHalfStep = Eigen::VectorXd::Zero(halfstep.size());
	fHalfStep.segment<3>(0) = halfstep.segment<3>(3);
	fHalfStep.segment<3>(3) = derivEval.segment<3>(3);

	return (state + timestep * fHalfStep);
}