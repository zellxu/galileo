#ifndef MIDPOINT_METHOD_H
#define MIDPOINT_METHOD_H

#include "solverinterface.h"

class MidpointMethod : public IntegratorInterface
{
public:
	virtual void integrate(SystemInterface* system, double timestep);
	virtual Eigen::VectorXd integrate(Eigen::VectorXd state, Eigen::VectorXd derivEval, double timestep);
};

#endif

