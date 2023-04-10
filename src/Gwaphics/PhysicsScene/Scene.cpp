#include "Scene.hpp"
#include <iostream>
#include <random>
#include <glm/gtx/string_cast.hpp>

Scene::Scene(Vulkan::CommandPool& commandPool)
{
	Model model("assets/sphere.obj", vertices, indices);
	std::cout << (vertices.size()) << std::endl;
	uint32_t cubeLen = 10;
	numParticles = cubeLen * cubeLen * cubeLen;
	/*positions.resize(numParticles);*/
	std::mt19937 generator(123);
	std::uniform_real_distribution<float> dis(0.0, 1.0);

	double randomRealBetweenZeroAndOne = dis(generator);
	glm::vec3 position(-0.5f);
	for (uint32_t i = 0; i < cubeLen; ++i)
	{
		for (uint32_t j = 0; j < cubeLen; ++j)
		{
			for (uint32_t k = 0; k < cubeLen; ++k)
			{
				positions.push_back(glm::vec4(position + 0.08f*glm::vec3(k, j, i), 0.f));
				velocities.push_back(glm::vec4(dis(generator), dis(generator), dis(generator), 0.f));
			}
		}
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

	MatrixXd currCOMPositions(2, 3); currCOMPositions << b1.currPos, b2.currPos;
	MatrixXd currConstPositions(2, 3); currConstPositions << penPosition + depth * contactNormal, penPosition;
	MatrixXd currCOMVelocities(2, 3); currCOMVelocities << b1.velocity, b2.velocity;

	Matrix3d zero = Matrix3d::Zero();
	MatrixXd correctedCOMVelocities, correctedAngVelocities, correctedCOMPositions;

	bool velocityWasValid = currConstraint.resolveVelocityConstraint(currCOMPositions, currConstPositions, currCOMVelocities, correctedCOMVelocities, tolerance, 0);

	if (!velocityWasValid) {
		//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
		b1.velocity = correctedCOMVelocities.row(0);
		b2.velocity = correctedCOMVelocities.row(1);
	}

	bool positionWasValid = currConstraint.resolvePositionConstraint(currCOMPositions, currConstPositions, correctedCOMPositions, tolerance, 0);

	if (!positionWasValid) {
		b1.currPos = correctedCOMPositions.row(0);
		b2.currPos = correctedCOMPositions.row(1);
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
	for (int i = 0; i < balls.size(); i++)
		balls[i].integrate(timeStep, dragCoeff);


	//detecting and handling collisions when found
	//This is done exhaustively: checking every two objects in the scene.
	double depth;
	RowVector3d contactNormal, penPosition;
	for (int i = 0; i < balls.size(); i++)
		for (int j = i + 1; j < balls.size(); j++)
			if (balls[i].isCollide(balls[j], depth, contactNormal, penPosition))
				handleCollision(balls[i], balls[j], balls[i].invMass, balls[j].invMass, depth, contactNormal, penPosition, CRCoeff, tolerance);


	//Resolving user constraints iteratively until either:
	//1. Positions or velocities are valid up to tolerance (a full streak of validity in the iteration)
	//2. maxIterations has run out


	//Resolving velocity
	int currIteration = 0;
	int zeroStreak = 0;  //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
	int currConstIndex = 0;
	while ((zeroStreak < constraints.size()) && (currIteration * constraints.size() < maxIterations)) {

		Constraint currConstraint = constraints[currConstIndex];

		RowVector3d currConstPos1 = balls[currConstraint.b1].currPos;
		RowVector3d currConstPos2 = balls[currConstraint.b2].currPos;

		MatrixXd currCOMPositions(2, 3); currCOMPositions << currConstPos1, currConstPos2;
		MatrixXd currConstPositions(2, 3); currConstPositions << currConstPos1, currConstPos2;
		MatrixXd currCOMVelocities(2, 3); currCOMVelocities << balls[currConstraint.b1].velocity, balls[currConstraint.b2].velocity;

		MatrixXd correctedCOMVelocities, correctedCOMPositions;

		bool velocityWasValid = currConstraint.resolveVelocityConstraint(currCOMPositions, currConstPositions, currCOMVelocities, correctedCOMVelocities, tolerance, flexCoeff);

		if (velocityWasValid) {
			zeroStreak++;
		}
		else {
			//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
			zeroStreak = 0;
			balls[currConstraint.b1].velocity = correctedCOMVelocities.row(0);
			balls[currConstraint.b2].velocity = correctedCOMVelocities.row(1);

		}

		currIteration++;
		currConstIndex = (currConstIndex + 1) % (constraints.size());
	}

	if (currIteration * constraints.size() >= maxIterations)
		cout << "Velocity Constraint resolution reached maxIterations without resolving!" << endl;


	//Resolving position ------------------- PROJECT
	currIteration = 0;
	zeroStreak = 0;  //how many consecutive constraints are already below tolerance without any change; the algorithm stops if all are.
	currConstIndex = 0;
	while ((zeroStreak < constraints.size()) && (currIteration * constraints.size() < maxIterations)) {

		Constraint currConstraint = constraints[currConstIndex];

		RowVector3d currConstPos1 = balls[currConstraint.b1].currPos;
		RowVector3d currConstPos2 = balls[currConstraint.b2].currPos;

		MatrixXd currCOMPositions(2, 3); currCOMPositions << currConstPos1, currConstPos2;
		MatrixXd currConstPositions(2, 3); currConstPositions << currConstPos1, currConstPos2;

		MatrixXd correctedCOMPositions;

		bool positionWasValid = currConstraint.resolvePositionConstraint(currCOMPositions, currConstPositions, correctedCOMPositions, tolerance, flexCoeff);

		if (positionWasValid) {
			zeroStreak++;
		}
		else {
			//only update the COM and angular velocity, don't both updating all currV because it might change again during this loop!
			zeroStreak = 0;

			balls[currConstraint.b1].currPos = correctedCOMPositions.row(0);
			balls[currConstraint.b2].currPos = correctedCOMPositions.row(1);

		}

		currIteration++;
		currConstIndex = (currConstIndex + 1) % (constraints.size());
	}

	if (currIteration * constraints.size() >= maxIterations)
		cout << "Position Constraint resolution reached maxIterations without resolving!" << endl;

	currTime += timeStep;
}

void Scene::addMesh(char const* filename) {
	BallLoader balls("assets/bunny.ball");
	for (int i = 0; i < balls.constraints.size(); i++) {
		double initDist = (this->balls[balls.constraints[i][0]].origPos - this->balls[balls.constraints[i][1]].origPos).norm();
		this->constraints.push_back(Constraint(DISTANCE, EQUALITY, this->balls.size() + balls.constraints[i][0], this->balls.size() + balls.constraints[i][1], this->balls[balls.constraints[i][0]].invMass, this->balls[balls.constraints[i][1]].invMass, RowVector3d::Zero(), initDist, 0.0));
	}

	for (int i = 0; i < balls.balls.size(); i++) {
		this->balls.push_back(Ball(100/balls.balls.size(), false, balls.balls[i]));
	}
	
}
