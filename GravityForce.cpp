#include "GravityForce.h"

GravityForce::GravityForce(std::vector<Particle*> particles): Force(particles),GRAVITY(0,-9.81,0){}

void GravityForce::apply() {
	for (int i = 0; i < mParticles.size(); i++)
		mParticles[i]->mAccumulatedForce += mParticles[i]->mMass * GRAVITY;
}