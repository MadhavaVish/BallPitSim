#include "Scene.hpp"
#include <iostream>
#include <random>
#include <glm/gtx/string_cast.hpp>

Scene::Scene(Vulkan::CommandPool& commandPool)
{
	Model model("assets/sphere.obj", vertices, indices);
	//std::cout << (vertices.size()) << std::endl;

	solver.reset(new SPHSolver(15, 0.02f, 1000, 1, 1.04f, 0.15f, -9.8f, 0.1f));
	solver->reset();

	numParticles = solver->particles.size();
	std::cout << numParticles;
	positions.resize(numParticles);
	/*positions.resize(numParticles);*/
	solver->getParticlePositions(positions);
	solver->startSimulation();
	std::mt19937 generator(123);
	std::uniform_real_distribution<float> dis(0.0, 1.0);


	constexpr auto flags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Vertices", VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flags, vertices, vertexBuffer_, vertexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Indices", VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flags, indices, indexBuffer_, indexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Positions", flags, positions, positionBuffer_, positionBufferMemory_);
}

Scene::~Scene()
{
	solver.reset();
	positionBuffer_.reset();
	positionBufferMemory_.reset();
	indexBuffer_.reset();
	indexBufferMemory_.reset();
	vertexBuffer_.reset();
	vertexBufferMemory_.reset();
}

void Scene::updatePositions(double deltaTime)
{
	solver->update(deltaTime);
	solver->getParticlePositions(positions);
	//std::cout << glm::to_string(solver->particles[0]->position) << std::endl;
}

void Scene::updateBuffer(Vulkan::CommandPool& commandPool)
{
	Vulkan::BufferUtil::CopyFromStagingBuffer(commandPool, *positionBuffer_, positions);
}
