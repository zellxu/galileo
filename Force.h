#ifndef FORCE_H
#define FORCE_H

#include <vector>
#include "Particle.h"

class Force
{
public:
	Force(std::vector<Particle*> particles);
	std::vector<Particle*> mParticles;
	virtual void apply() = 0;
};

#endif