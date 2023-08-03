#include "Scene.hpp"
#include <iostream>
#include <random>
#include <glm/gtx/string_cast.hpp>

Scene::Scene(Vulkan::CommandPool& commandPool)
{
	Model model("assets/sphere.obj", vertices, indices);
	std::cout << (vertices.size()) << std::endl;
	addBunny();
	addFloor();
	//addCalibCube();
	numParticles = this->balls.size();
	currTime = 0;
	currStep = 0;
	/*positions.resize(numParticles);*/
	std::mt19937 generator(123);
	std::uniform_real_distribution<float> dis(0.0, 1.0);

	for (uint32_t i = 0; i < numParticles; ++i)
	{
		Ball currBall = this->balls[i];
		positions.push_back(glm::vec4(glm::vec3(currBall.predictedP[0], -currBall.predictedP[1], currBall.predictedP[2]), 0.f));
		velocities.push_back(glm::vec4(0.f, 0.f, 0.f, 0.f));
	}
	//for (auto position : positions)
	//	std::cout << glm::to_string(position) << std::endl;
	std::cout << positions.size() << std::endl;
	constexpr auto flags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Vertices", VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flags, vertices, vertexBuffer_, vertexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Indices", VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flags, indices, indexBuffer_, indexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Positions", flags, positions, positionBuffer_, positionBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Colors", flags, ballsColor, colorBuffer_, colorBufferMemory_);
}

Scene::~Scene()
{
	colorBuffer_.reset();
	colorBufferMemory_.reset();
	positionBuffer_.reset();
	positionBufferMemory_.reset();
	indexBuffer_.reset();
	indexBufferMemory_.reset();
	vertexBuffer_.reset();
	vertexBufferMemory_.reset();
}

void Scene::updatePositions(double deltaTime)
{
	currTime += deltaTime;
	if (currTime > 1)
		isRunning = true;
	
	if (isRunning)
		currStep += deltaTime;

	if (currStep > physicsStepTime) {
		updateScene(physicsStepTime, CRCoeff, dragCoeff, tolerance, maxIterations);
		currStep = 0;
	}
}

void Scene::updateBuffer(Vulkan::CommandPool& commandPool)
{
	Vulkan::BufferUtil::CopyFromStagingBuffer(commandPool, *positionBuffer_, positions);
}

void Scene::handleCollision(Constraint c) {
	/*
		Update m(1,2) comVelocity, angVelocity and COM variables by using a Constraint class of type COLLISION
	*/
	Ball& ball1 = balls[c.b1];
	Ball& ball2 = balls[c.b2];
	RowVector3d penPosition = ball1.predictedP + c.refVector.normalized() * ball1.radius;

	Matrix<double,2,3> currBallPositions; currBallPositions << ball1.predictedP, ball2.predictedP;
	Matrix<double, 2, 3> currConstPositions; currConstPositions << penPosition + c.refValue * c.refVector, penPosition;
	Matrix<double, 2, 3> currBallVelocities; currBallVelocities << ball1.velocity, ball2.velocity;

	MatrixXd deltaValVel, deltaValPos;

	// TODO : Suggested method store lastPos and currPos and update velocity according to the difference between those and not updateVel here

	bool velocityWasValid = c.resolveVelocityConstraint(currBallPositions, currConstPositions, currBallVelocities, deltaValVel, tolerance);

	if (!velocityWasValid) {

		ball1.dv += deltaValVel.row(0);
		ball2.dv += deltaValVel.row(1);

	}

	bool positionWasValid = c.resolvePositionConstraint(currBallPositions, currConstPositions, deltaValPos, tolerance);

	if (!positionWasValid) {
		ball1.dx += deltaValPos.row(0);
		ball2.dx += deltaValPos.row(1);
	}

	if (!velocityWasValid || !positionWasValid) { ball1.n++; ball2.n++; }

}

/*********************************************************************
	 This function handles a single time step by:
	 1. Integrating velocities, positions, and orientations by the timeStep
	 2. (Practical 2) Detecting collisions and encoding them as constraints
	 3. (Practical 2) Iteratively resolved positional and velocity constraints

	 You do not need to update this function in Practical 2
	 *********************************************************************/
void Scene::updateScene(const double timeStep, const double CRCoeff, const double dragCoeff, const double tolerance, const int maxIterations) {
	vector<Constraint> contactConstraints;
	//integrating velocity, position and orientation from forces and previous states
	for (int i = 0; i < balls.size(); i++) {
		balls[i].integrate(timeStep, dragCoeff);
	}
		
	// Detecting and handling collisions when found
	// This is done exhaustively: checking every two objects in the scene.
	// TODO (5) : Spatial partitionning
	double depth;
	RowVector3d contactNormal, penPosition;
	for (int i = 0; i < meshes.size(); i++) {
		for (int j = i + 1; j < meshes.size(); j++) {
			for (int k = meshes[i].globalOffset; k < meshes[i].globalOffset + meshes[i].length; k++) {		// TODO (5) : For all Balls
				for (int l = meshes[j].globalOffset; l < meshes[j].globalOffset + meshes[j].length; l++) {	// TODO (5) : Spatial partitionning => Find Neighboring function
					if (balls[k].isCollide(balls[l], depth, contactNormal, penPosition)) {
						contactConstraints.push_back(Constraint(COLLISION, INEQUALITY, k, l, balls[k].invScaleMass(), balls[l].invScaleMass(), contactNormal, depth, CRCoeff));
						break;
					}
				}
			}
		}
	}

	//Resolving Collisions
	int currIteration = 0;
	int zeroStreak = 0;  //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
	int currConstIndex = 0;
	while ((zeroStreak < contactConstraints.size()) && (currIteration < maxIterations)) {
		for (int i = 0; i < contactConstraints.size(); i++) {
			Constraint currConst = contactConstraints[i];
			handleCollision(currConst);
		}
		for (int i = 0; i < contactConstraints.size(); i++) {
			Constraint currConst = contactConstraints[i];
			balls[currConst.b1].resolveContact(timeStep);
			balls[currConst.b2].resolveContact(timeStep);
			if (!balls[currConst.b1].isCollide(balls[currConst.b2], depth, contactNormal, penPosition)) zeroStreak++;
		}
		currIteration++;
		currConstIndex = (currConstIndex + 1) % (contactConstraints.size());
	}
	contactConstraints.clear();

	//Resolving user constraints iteratively until either:
	//1. Positions or velocities are valid up to tolerance (a full streak of validity in the iteration)
	//2. maxIterations has run out

	//Resolving velocity & position
	currIteration = 0;
	zeroStreak = 0;  //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
	currConstIndex = 0;
	while ((zeroStreak < constraints.size()) && (currIteration < maxIterations)) {

		for (int i = 0; i < constraints.size(); i++) {
			Constraint c = constraints[i];

			Ball& ball1 = balls[c.b1];
			Ball& ball2 = balls[c.b2];

			Matrix<double, 2, 3> currBallPositions; currBallPositions << ball1.predictedP, ball2.predictedP;
			Matrix<double, 2, 3> currConstPositions; currConstPositions << ball1.predictedP, ball2.predictedP;
			Matrix<double, 2, 3> currBallVelocities; currBallVelocities << ball1.velocity, ball2.velocity;

			MatrixXd deltaValVel, deltaValPos;

			// TODO : Suggested method store lastPos and currPos and update velocity according to the difference between those and not updateVel here

			bool velocityWasValid = c.resolveVelocityConstraint(currBallPositions, currConstPositions, currBallVelocities, deltaValVel, tolerance);

			if (!velocityWasValid) {
				zeroStreak = 0;
				ball1.dv += deltaValVel.row(0);
				ball2.dv += deltaValVel.row(1);
			}

			bool positionWasValid = c.resolvePositionConstraint(currBallPositions, currConstPositions, deltaValPos, tolerance);

			if (!positionWasValid) {
				zeroStreak = 0;

				ball1.dx += deltaValPos.row(0);
				ball2.dx += deltaValPos.row(1);
			}

			if (velocityWasValid && positionWasValid) zeroStreak++;
			else { ball1.n++; ball2.n++; }
		}

		for (int i = 0; i < constraints.size(); i++) {
			Constraint currConst = constraints[i];
			balls[currConst.b1].resolve(timeStep);
			balls[currConst.b2].resolve(timeStep);
		}

		currIteration++;
		currConstIndex = (currConstIndex + 1) % (constraints.size());

	}
	if (currIteration * constraints.size() >= maxIterations)
		cout << "Constraint solving reached maxIterations !" << endl; 

	for (int i = 0; i < balls.size(); i++) {
	/*	update velocity vi <= 1 / dt * (xi_new - xi)
		advect diffuse particles
		apply internal forces fdrag, fvort */
		balls[i].dampVelocity();
		if ((balls[i].predictedP - balls[i].pos).norm() < 1e-4) balls[i].pos = balls[i].pos; else balls[i].pos = balls[i].predictedP;
		velocities[i] = glm::vec4(balls[i].velocity[0], -balls[i].velocity[1], balls[i].velocity[2], 0.0f);
		positions[i] = glm::vec4(balls[i].predictedP[0], -balls[i].predictedP[1], balls[i].predictedP[2], 0.0f);
	}
}

void Scene::addBunny() {
	BallLoader balls("assets/bunny.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0.75,2,0.75);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(BallType::RigidBody, 0, balls.balls.size() / 100.0, false, (balls.balls[i]/10)+ translation, balls.normals[i]));
		this->ballsColor.push_back(glm::vec4(0.93, 0.85, 0.33,1.0));
	}

	for (int i = 0; i < balls.constraints.size(); i++) {
		double initDist = (this->balls[balls.constraints[i][0]].pos - this->balls[balls.constraints[i][1]].pos).norm();
		this->constraints.push_back(Constraint(DISTANCE, EQUALITY, initialBallNb + balls.constraints[i][0], initialBallNb + balls.constraints[i][1], balls.balls.size() / 100.0, balls.balls.size() / 100.0, (this->balls[initialBallNb + balls.constraints[i][1]].predictedP - this->balls[initialBallNb + balls.constraints[i][0]].predictedP).normalized(), initDist, 0.0));
	}
}

void Scene::addPool() {
	BallLoader balls("assets/pool.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0, 0, 0);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(BallType::RigidBody, 1, 0, true, (balls.balls[i] / 10) + translation, balls.normals[i]));
		this->ballsColor.push_back(glm::vec4(0.93, 0.33, 0.33, 1.0));
	}
}

void Scene::addFloor() {
	BallLoader balls("assets/floor.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0, 0, 0);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(BallType::RigidBody, 1, 0, true, (balls.balls[i] / 10) + translation, balls.normals[i]));
		this->ballsColor.push_back(glm::vec4(0.93, 0.33, 0.33, 1.0));
	}
}

void Scene::addCalibCube() {
	BallLoader balls("assets/calibCube.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0, 0, 0);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(BallType::RigidBody, 1, 0, true, (balls.balls[i] / 10) + translation, RowVector3d::Zero() ));
		this->ballsColor.push_back(glm::vec4(0.93, 0.33, 0.33, 1.0));
	}
}