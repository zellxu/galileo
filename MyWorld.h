#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "SolverInterface.h"
#include "Force.h"

class Particle;

class MyWorld : public SystemInterface{
public:
	MyWorld(int _numParticles);

	virtual ~MyWorld();

	int getNumParticles(){
		return mParticles.size();
	}

	Particle* getParticle(int _index) {
		return mParticles[_index];
	}

	// TODO: your simulation code goes here
	void simulate();
	
	virtual void setState(const Eigen::VectorXd& state);
	virtual Eigen::VectorXd getState();
	virtual Eigen::VectorXd derivEval();
	virtual void clearForces();
	virtual void calculateForces();
	
protected:
	std::vector<Particle*> mParticles;
	double mTime;
	double mTimestep;
	std::vector<Force*> mForces;
	IntegratorInterface* mIntegrator;
};

#endif
