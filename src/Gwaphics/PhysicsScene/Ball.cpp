#include "Ball.hpp"

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
	
	if ((isFixed && b.isFixed) || (meshId == b.meshId))  return false; //collision does nothing
		
	if (type == BallType::RigidBody && b.type == BallType::RigidBody) {
		RowVector3d collisionVec = b.predictedP - predictedP;
		if (collisionVec.norm() > 2 * radius) return false;
		
		if (isFixed || normal.norm() > b.normal.norm())
			contactNormal = normal;
		if (b.isFixed || b.normal.norm() > normal.norm())
			contactNormal = b.normal;
		if (!isFixed && !b.isFixed) contactNormal = contactNormal.normalized();

		contactNormal *= contactNormal.dot(collisionVec);

		depth = 2 * radius - collisionVec.norm();
		penPosition = predictedP + collisionVec * (radius - depth);

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

void Ball::resolve(double timeStep) {
	if (n != 0) {
		predictedP += dx / n;
		pos += dx / n;
		velocity += (dv / n);// +((predictedP - pos) / timeStep);
	}

	dx = RowVector3d::Zero();
	dv = RowVector3d::Zero();
	n = 0;
}

void Ball::resolvePredicted(double timeStep) {
	if (n != 0) {
		predictedP += dx / n;
		velocity += (dv / n);// +((predictedP - pos) / timeStep);
	}

	dx = RowVector3d::Zero();
	dv = RowVector3d::Zero();
	n = 0;
}