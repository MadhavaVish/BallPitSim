#include "Ball.hpp"

bool Ball::isCollide(const Ball& b, double& depth, RowVector3d& intNormal, RowVector3d& intPosition) {

	if (meshId == b.meshId)  //collision does nothing
		return false;

	if ((isFixed && b.isFixed))  //collision does nothing
		return false;

	intNormal = (b.currPos - this->currPos);
	
	if (isFixed)
		intNormal = normal;

	if (b.isFixed)
		intNormal = b.normal;

	intNormal = intNormal.normalized();

	RowVector3d collisionVec = b.currPos - this->currPos;
	double dist = collisionVec.norm();

	if (dist > 2*radius)
		return false;

	depth = 2 * radius - collisionVec.norm();
	intPosition = this->currPos + collisionVec.normalized() * (radius - depth);

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

	velocity -= dragCoeff * velocity * timeStep;
}


//the full integration for the time step (velocity + position)
//You need to modify this if you are changing the integration
void Ball::integrate(double timeStep, const double dragCoeff) {
	updateVelocity(timeStep, dragCoeff);
	updatePosition(timeStep);
}