#ifndef SOLVER_INTERFACE_H
#define SOLVER_INTERFACE_H

#include <Eigen/Dense>

class SystemInterface {
public:
	virtual void			setState(const Eigen::VectorXd& state)	= 0;
	virtual Eigen::VectorXd	getState()								= 0;
	virtual Eigen::VectorXd derivEval()								= 0;
};

class IntegratorInterface {
public:	
	virtual void			integrate(SystemInterface* system, double timestep)								= 0;
	virtual Eigen::VectorXd integrate(Eigen::VectorXd state, Eigen::VectorXd derivEval, double timestep)	= 0;
};

#endif