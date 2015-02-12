#include "MyWorld.h"
#include "Particle.h"
#include "GravityForce.h"
#include "ExplicitEuler.h"
#include "MidpointMethod.h"

using namespace Eigen;

MyWorld::MyWorld(int _numParticles) {
	mTime = 0.0;
	mTimestep = 0.005;

	// Create particles
	for (int i = 0; i < _numParticles; i++) {
		Particle *p = new Particle();
		mParticles.push_back(p);
	}

	mForces.push_back(new GravityForce(mParticles));

	// Init particle positions (default is 0, 0, 0)
	mParticles[0]->mPosition[0] = -0.3;
	mParticles[2]->mPosition[0] = 0.3;
	
	// Init particle heights
	double initHeight = 80.0;
	mParticles[0]->mPosition[1] = initHeight;
	mParticles[1]->mPosition[1] = initHeight;
	mParticles[2]->mPosition[1] = initHeight;

	// Init particle colors (default is red)
	mParticles[1]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
	mParticles[2]->mColor = Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
}

MyWorld::~MyWorld() {
	for (int i = 0; i < mParticles.size(); i++)
		delete mParticles[i];
	mParticles.clear();
}
#include <iostream>

void MyWorld::simulate() {

	Eigen::VectorXd result;

	//analytic solution
	mParticles[0]->mPosition[1] = 80 +  mParticles[0]->mVelocity[1] * mTime + (-9.81)/2.0 * mTime * mTime;
	mParticles[0]->mVelocity[1] = (-9.81) * mTimestep;

	//Explicit Euler
	mIntegrator = new ExplicitEuler();
	result = mIntegrator->integrate(getState().segment<6>(6),derivEval().segment<6>(6),mTimestep);
	mParticles[1]->mPosition = result.segment<3>(0);
	mParticles[1]->mVelocity = result.segment<3>(3);

	//Midpoint Method
	mIntegrator = new MidpointMethod();
	result = mIntegrator->integrate(getState().segment<6>(12),derivEval().segment<6>(12),mTimestep);
	mParticles[2]->mPosition = result.segment<3>(0);
	mParticles[2]->mVelocity = result.segment<3>(3);

	mTime += mTimestep;
}

void MyWorld::setState(const Eigen::VectorXd& state){

	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mPosition = state.segment<3>(6*i);
		mParticles[i]->mVelocity = state.segment<3>(6*i+3);
	}
}

Eigen::VectorXd	MyWorld::getState(){

	Eigen::VectorXd state = Eigen::VectorXd::Zero(mParticles.size() * 6);
	for (int i = 0; i < mParticles.size(); i++) {
		state.segment<3>(6*i) = mParticles[i]->mPosition;
		state.segment<3>(6*i+3) = mParticles[i]->mVelocity;
	}
	return state;
}

Eigen::VectorXd MyWorld::derivEval(){
	clearForces();
	calculateForces();
	
	Eigen::VectorXd deriv = Eigen::VectorXd::Zero(mParticles.size() * 6);
	for (int i = 0; i < mParticles.size(); i++) {
		deriv.segment<3>(6*i) = mParticles[i]->mVelocity;
		deriv.segment<3>(6*i+3) = mParticles[i]->mAccumulatedForce/mParticles[i]->mMass;
	}

	return deriv;
}

void MyWorld::clearForces() {
	for (int i = 0; i < mParticles.size(); i++)
		mParticles[i]->mAccumulatedForce.setZero();
}

void MyWorld::calculateForces() {
	for (int i = 0; i < mForces.size(); i++)
		mForces[i]->apply();
}