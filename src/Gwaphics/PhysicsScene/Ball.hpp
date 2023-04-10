#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

class Ball {
public:
	const double radius = 0.2;
	RowVector3d origPos;   //original vertex positions, where COM=(0.0,0.0,0.0) - never change this!
	RowVector3d currPos;   //current vertex position
	RowVector3d velocity;

	//kinematics
	bool isFixed;  //is the object immobile
	double invMass;

	Ball(const double _mass, const bool _isFixed, const RowVector3d& _Pos) {
		invMass = 1 / _mass;
		origPos = _Pos;
		currPos = _Pos;
		isFixed = _isFixed;
		velocity.setZero();
	}
	~Ball() {}

	bool isCollide(const Ball& m, double& depth, RowVector3d& intNormal, RowVector3d& intPosition);
	void integrate(double timeStep, const double dragCoeff);
	void updatePosition(double timeStep);
	void updateVelocity(double timeStep, const double dragCoeff);
};