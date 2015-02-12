#ifndef EXPLICIT_EULER_H
#define EXPLICIT_EULER_H

#include "solverinterface.h"

class ExplicitEuler : public IntegratorInterface
{
public:
	virtual void integrate(SystemInterface* system, double timestep);
	virtual Eigen::VectorXd integrate(Eigen::VectorXd state, Eigen::VectorXd derivEval, double timestep);
};

#endif