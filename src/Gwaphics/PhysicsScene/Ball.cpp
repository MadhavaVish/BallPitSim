#include "Ball.hpp"

bool Ball::isCollide(const Ball& b, double& depth, RowVector3d& intNormal, RowVector3d& intPosition) {
	
	double r = this->radius;

	if (meshId == b.meshId)  //collision does nothing
		return false; //r=0.05

	if ((isFixed && b.isFixed))  //collision does nothing
		return false;

	intNormal = (b.currPos - this->currPos);
	intNormal = intNormal.normalized();

	if (isFixed)
		intNormal = normal;

	if (b.isFixed)
		intNormal = b.normal;

	RowVector3d collisionVec = b.currPos - this->currPos;
	double dist = collisionVec.norm();

	if (isFixed || b.isFixed) {
		collisionVec = intNormal * intNormal.dot(collisionVec);
	}

	if (dist > 2*r)
		return false;

	depth = 2 * r - collisionVec.norm();
	intPosition = this->currPos + collisionVec.normalized() * (r - depth);

	return true;
}


//Update the current position and orientation by integrating the linear and angular velocities, and update currV accordingly
//You need to modify this according to its purpose
void Ball::updatePosition(double timeStep) {
	
	if (isFixed)
		return;  // Not moving fixed object

	//Forward Euler now
	if (timeStep > 0) {;
		currPos << currPos + velocity * timeStep;
	}
}


//Updating the linear and angular velocities of the object
//You need to modify this to integrate from acceleration in the field (basically gravity)
void Ball::updateVelocity(double timeStep, const double dragCoeff) {

	if (isFixed)
		return; // Not moving fixed object

	Vector3d gravity; gravity << 0, -0.098, 0.0;
	velocity += gravity * timeStep;

	velocity =  (1 - (dragCoeff * timeStep)) * velocity;
}


//the full integration for the time step (velocity + position)
//You need to modify this if you are changing the integration
void Ball::integrate(double timeStep, const double dragCoeff) {
	updateVelocity(timeStep, dragCoeff);
	updatePosition(timeStep);
}