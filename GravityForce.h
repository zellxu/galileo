#ifndef GRAVITY_FORCE_H
#define GRAVITY_FORCE_H

#include <Eigen/Dense>
#include "Force.h"

class GravityForce : public Force{
private:
	Eigen::Vector3d GRAVITY; 

public:
	GravityForce(std::vector<Particle*> particles);
	virtual void apply();
};

#endif