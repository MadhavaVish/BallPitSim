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
	const double radius = 0.1;
	RowVector3d origPos;   //original ball positions
	RowVector3d currPos;   //current ball position
	RowVector3d velocity;
	RowVector3d normal;

	//kinematics
	bool isFixed;  //is the object immobile
	double invMass;

	Ball(const int _meshId, const double _invMass, const bool _isFixed, const RowVector3d& _Pos, const RowVector3d& _Norm) {
		invMass = _invMass;
		origPos = _Pos;
		currPos = _Pos;
		normal = _Norm;
		isFixed = _isFixed;
		meshId = _meshId;
		velocity.setZero();
	}
	~Ball() {}

	bool isCollide(const Ball& b, double& depth, RowVector3d& intNormal, RowVector3d& intPosition);
	void integrate(double timeStep, const double dragCoeff);
	void updatePosition(double timeStep);
	void updateVelocity(double timeStep, const double dragCoeff);
private:
	int meshId;
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