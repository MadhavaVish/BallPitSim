#include "Scene.hpp"
#include <iostream>
#include <random>
#include <glm/gtx/string_cast.hpp>

Scene::Scene(Vulkan::CommandPool& commandPool)
{
	Model model("assets/sphere.obj", vertices, indices);
	std::cout << (vertices.size()) << std::endl;
	addBunny();
	addPool();
	numParticles = this->balls.size();
	currTime = 0;
	currStep = 0;
	physicsStepTime = 0.2;
	/*positions.resize(numParticles);*/
	std::mt19937 generator(123);
	std::uniform_real_distribution<float> dis(0.0, 1.0);

	for (uint32_t i = 0; i < numParticles; ++i)
	{
		Ball currBall = this->balls[i];
		positions.push_back(glm::vec4(glm::vec3(currBall.currPos[0], -currBall.currPos[1], currBall.currPos[2]), 0.f));
		velocities.push_back(glm::vec4(0.f, 0.f, 0.f, 0.f));
	}
	//for (auto position : positions)
	//	std::cout << glm::to_string(position) << std::endl;
	//std::cout << positions.size() << std::endl;
	constexpr auto flags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Vertices", VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flags, vertices, vertexBuffer_, vertexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Indices", VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flags, indices, indexBuffer_, indexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Positions", flags, positions, positionBuffer_, positionBufferMemory_);
}

Scene::~Scene()
{
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
	if (currTime > 3)
		isRunning = true;
	
	if (isRunning)
		currStep += deltaTime;

	if (currStep > physicsStepTime) {
		updateScene(physicsStepTime, CRCoeff, dragCoeff, tolerance, maxIterations, flexCoeff);
		currStep = 0;
	}

	for (size_t i = 0; i < positions.size(); ++i)
	{
		positions[i] += velocities[i] * (float)deltaTime;
	}
}

void Scene::updateBuffer(Vulkan::CommandPool& commandPool)
{
	Vulkan::BufferUtil::CopyFromStagingBuffer(commandPool, *positionBuffer_, positions);
}

void Scene::handleCollision(Ball& b1, Ball& b2, const double& invMass1, const double& invMass2, const double& depth, const RowVector3d& contactNormal, const RowVector3d& penPosition, const double CRCoeff, const double tolerance) {
	/***************
	 TODO: practical 2
	 update m(1,2) comVelocity, angVelocity and COM variables by using a Constraint class of type COLLISION
	 ***********************/

	Constraint currConstraint = Constraint(COLLISION, INEQUALITY, 0, 0, invMass1, invMass2, contactNormal, depth, CRCoeff);

	Matrix<double,2,3> currBallPositions; currBallPositions << b1.currPos, b2.currPos;
	Matrix<double, 2, 3> currConstPositions; currConstPositions << penPosition + depth * contactNormal, penPosition;
	Matrix<double, 2, 3> currBallVelocities; currBallVelocities << b1.velocity, b2.velocity;

	MatrixXd correctedBallVelocities, correctedBallPositions;

	bool velocityWasValid = currConstraint.resolveVelocityConstraint(currBallPositions, currConstPositions, currBallVelocities, correctedBallVelocities, tolerance, 0);

	if (!velocityWasValid) {
		//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
		b1.velocity = correctedBallVelocities.row(0);
		b2.velocity = correctedBallVelocities.row(1);
	}

	bool positionWasValid = currConstraint.resolvePositionConstraint(currBallPositions, currConstPositions, correctedBallPositions, tolerance, 0);

	if (!positionWasValid) {
		b1.currPos = correctedBallPositions.row(0);
		b2.currPos = correctedBallPositions.row(1);
	}

}

/*********************************************************************
	 This function handles a single time step by:
	 1. Integrating velocities, positions, and orientations by the timeStep
	 2. (Practical 2) Detecting collisions and encoding them as constraints
	 3. (Practical 2) Iteratively resolved positional and velocity constraints

	 You do not need to update this function in Practical 2
	 *********************************************************************/
void Scene::updateScene(const double timeStep, const double CRCoeff, const double dragCoeff, const double tolerance, const int maxIterations, const float flexCoeff) {

	//integrating velocity, position and orientation from forces and previous states
	for (int i = 0; i < balls.size(); i++) {
		balls[i].integrate(timeStep, dragCoeff);
	}
		
	//detecting and handling collisions when found
	//This is done exhaustively: checking every two objects in the scene.
	double depth;
	RowVector3d contactNormal, penPosition;
	for (int i = 0; i < meshes.size(); i++) {
		for (int j = i + 1; j < meshes.size(); j++) {
			for (int k = meshes[i].globalOffset; k < meshes[i].globalOffset + meshes[i].length; k++) {
				for (int l = meshes[j].globalOffset; l < meshes[j].globalOffset + meshes[j].length; l++) {
					if (balls[k].isCollide(balls[l], depth, contactNormal, penPosition)) {
						handleCollision(balls[k], balls[l], balls[k].invMass, balls[l].invMass, depth, contactNormal, penPosition, CRCoeff, tolerance);
						break;
					}
				}
			}
		}
	}

	for (int i = 0; i < balls.size(); i++) {
		velocities[i] = glm::vec4(balls[i].velocity[0], -balls[i].velocity[1], balls[i].velocity[2], 0.0f);
		positions[i] = glm::vec4(balls[i].currPos[0], -balls[i].currPos[1], balls[i].currPos[2], 0.0f);
	}
}

void Scene::addBunny() {
	BallLoader balls("assets/bunny.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0.75,3,0.75);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(0, balls.balls.size() / 100.0, false, (balls.balls[i]/10)+ translation, balls.normals[i]));
		this->ballsCol.push_back(glm::vec4(0.93, 0.85, 0.33,1.0));
	}

	for (int i = 0; i < balls.constraints.size(); i++) {
		double initDist = (this->balls[balls.constraints[i][0]].origPos - this->balls[balls.constraints[i][1]].origPos).norm();
		this->constraints.push_back(Constraint(DISTANCE, EQUALITY, initialBallNb + balls.constraints[i][0], initialBallNb + balls.constraints[i][1], balls.balls.size() / 100, balls.balls.size() / 100, this->balls[balls.constraints[i][1]].currPos - this->balls[balls.constraints[i][0]].currPos, initDist, 0.0));
		this->ballsCol.push_back(glm::vec4(0.93, 0.33, 0.33, 1.0));
	}
	
}

void Scene::addPool() {
	BallLoader balls("assets/pool.ball");
	int initialBallNb = this->balls.size();
	meshes.push_back(Mesh(initialBallNb, balls.balls.size()));
	RowVector3d translation = Vector3d(0, 0, 0);
	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(1, 0, true, (balls.balls[i] / 10) + translation, balls.normals[i]));
	}
}
