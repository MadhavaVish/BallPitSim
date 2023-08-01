#include <vector>
#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

typedef enum BallType { RigidBody, Water, Smoke };

class Ball {
public:
	const double radius = 5e-2;
	const double insideRadius = 4e-2;
	RowVector3d pos;
	RowVector3d predictedP;
	RowVector3d velocity;
	BallType type;
	RowVector3d normal;
	int meshId;

	//kinematics
	bool isFixed;  //is the object immobile
	double invMass;
	RowVector3d dx;
	RowVector3d dv;
	int n;
	
	

	Ball(const BallType _type, const int _meshId, const double _invMass, const bool _isFixed, const RowVector3d& _Pos, const RowVector3d& _Norm) {
		type = _type;
		invMass = _invMass;
		pos = _Pos;
		predictedP = _Pos;
		normal = _Norm;
		isFixed = _isFixed;
		meshId = _meshId;
		velocity.setZero();
		dx = RowVector3d::Zero();
		dv = RowVector3d::Zero();
		n = 0;
	}
	~Ball() {}

	double invScaleMass();
	bool isCollide(const Ball& b, double& depth, RowVector3d& intNormal, RowVector3d& intPosition);
	void integrate(double timeStep, const double dragCoeff);
	void resolve(double timeStep);
	void resolvePredicted(double timeStep);
	void updatePosition(double timeStep);
	void updateVelocity(double timeStep, const double dragCoeff);
};

class Mesh {
public:
	int globalOffset;
	int length;

	Mesh(int _globalOffset, int _length) {
		globalOffset = _globalOffset;
		length = _length;
	}
	~Mesh() {}
};