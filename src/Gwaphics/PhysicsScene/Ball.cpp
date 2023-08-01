#include "Ball.hpp"

void Ball::resetDelta() {
	dx = { 0, 0, 0 };
	n = 0;
}

double Ball::invScaleMass() { 
	double k = 1; 
	return invMass * ( 1 / exp(-k * predictedP(1)) ); 
}

//the full integration for the time step (velocity + position)
//You need to modify this if you are changing the integration
void Ball::integrate(double timeStep, const double dragCoeff) {
	updateVelocity(timeStep, dragCoeff);
	updatePosition(timeStep);
}

bool Ball::isCollide(const Ball& b, double& depth, RowVector3d& contactNormal, RowVector3d& penPosition) {
	
	if (isFixed && b.isFixed)  return false; //collision does nothing
		
	double r = ( meshId == b.meshId? insideRadius : radius ); //collision is respecting a smaller radius in order to lower collision detection within same mesh
	
	if (type == BallType::RigidBody && b.type == BallType::RigidBody) {
		RowVector3d collisionVec = b.predictedP - predictedP;
		if (collisionVec.norm() > 2 * r) return false;
		
		if (isFixed || normal.norm() > b.normal.norm())
			contactNormal = normal;
		if (b.isFixed || b.normal.norm() > normal.norm())
			contactNormal = b.normal;
		if (!isFixed && !b.isFixed) contactNormal = contactNormal.normalized();

		contactNormal *= contactNormal.dot(collisionVec);

		depth = 2 * r - collisionVec.norm();
		penPosition = predictedP + collisionVec * (r - depth);

		return true;
	}

	return false;
}

//Updating the linear and angular velocities of the object
//You need to modify this to integrate from acceleration in the field (basically gravity)
void Ball::updateVelocity(double timeStep, const double dragCoeff) {

	if (isFixed)
		return; // Not moving fixed object

	Vector3d gravity; gravity << 0, -0.098, 0.0;
	velocity += gravity * timeStep;

	velocity *= 1 - (dragCoeff * timeStep);
}

//Update the current position and orientation by integrating the linear and angular velocities, and update currV accordingly
//You need to modify this according to its purpose
void Ball::updatePosition(double timeStep) {

	if (isFixed)
		return;  // Not moving fixed object

	predictedP << predictedP + velocity * timeStep;
}