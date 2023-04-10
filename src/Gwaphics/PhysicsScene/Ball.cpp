#include "Ball.hpp"

bool Ball::isCollide(const Ball& b, double& depth, RowVector3d& intNormal, RowVector3d& intPosition) {


	if ((isFixed && b.isFixed))  //collision does nothing
		return false;

	RowVector3d collisionVec = this->currPos - b.currPos;
	double dist = fabs(collisionVec.norm() );

	if (dist > 2*radius)
		return false;

	intNormal = collisionVec.normalized();

	depth = 2 * radius - collisionVec.norm();
	intPosition = b.currPos + collisionVec.normalized() * (radius - depth);


	return true;

}


//Update the current position and orientation by integrating the linear and angular velocities, and update currV accordingly
//You need to modify this according to its purpose
void Ball::updatePosition(double timeStep) {
	//just forward Euler now
	if (isFixed)
		return;  //a fixed object is immobile

	if (timeStep > 0) {;
		currPos << currPos + velocity * timeStep;
	}
}


//Updating the linear and angular velocities of the object
//You need to modify this to integrate from acceleration in the field (basically gravity)
void Ball::updateVelocity(double timeStep, const double dragCoeff) {

	if (isFixed)
		return;

	Vector3d gravity; gravity << 0, -9.8, 0.0;
	velocity += gravity * timeStep;

	velocity -= dragCoeff * velocity * timeStep;
}


//the full integration for the time step (velocity + position)
//You need to modify this if you are changing the integration
void Ball::integrate(double timeStep, const double dragCoeff) {
	updateVelocity(timeStep, dragCoeff);
	updatePosition(timeStep);
}