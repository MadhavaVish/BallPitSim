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
