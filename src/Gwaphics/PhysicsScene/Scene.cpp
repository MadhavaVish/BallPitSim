#include "Scene.hpp"
#include <iostream>
#include <random>
#include <glm/gtx/string_cast.hpp>
static glm::vec3 randomPointInSphere(float d1, float d2, float d3)
{
	float theta = d1 * glm::two_pi<float>();
	float phi = glm::acos(2.0 * d2 - 1.0);
	float r = glm::pow(d3, 0.3333333f);
	float sinTheta = glm::sin(theta);
	float cosTheta = glm::cos(theta);
	float sinPhi = glm::sin(phi);
	float cosPhi = glm::cos(phi);
	float x = r * sinPhi * cosTheta;
	float y = r * sinPhi * sinTheta;
	float z = r * cosPhi;
	return glm::vec3(x, y, z);
}
Scene::Scene(Vulkan::CommandPool& commandPool)
{
	Model model("assets/sphere.obj", vertices, indices);
	std::cout << (vertices.size()) << std::endl;
	uint32_t cubeLen = 10;
	numParticles = 200;
	/*positions.resize(numParticles);*/
	std::mt19937 generator(123);
	std::uniform_real_distribution<float> dis(0.0, 1.0);

	//double randomRealBetweenZeroAndOne = dis(generator);
	//glm::vec3 position(-0.5f);
	for (uint32_t i = 0; i < numParticles; ++i)
	{
		Vertex vert;
		glm::vec4 point = 60.f * glm::vec4(randomPointInSphere(dis(generator), dis(generator), dis(generator)), 0.f);
		positions.push_back(point);
		glm::vec3 v = 2.f * glm::vec3(dis(generator), dis(generator), dis(generator)) - 1.f;
		velocities.push_back(8.f*glm::vec4(v,0.f));
		masses.push_back(dis(generator) * 500000 + 100000);
		
	}
	for (auto vel : velocities)
	{
		speeds.push_back(vel.length());
	}
	//for (auto position : positions)
	//	std::cout << glm::to_string(position) << std::endl;
	//std::cout << positions.size() << std::endl;
	constexpr auto flags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Vertices", VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | flags, vertices, vertexBuffer_, vertexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Indices", VK_BUFFER_USAGE_INDEX_BUFFER_BIT | flags, indices, indexBuffer_, indexBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Positions", flags, positions, positionBuffer_, positionBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Masses", flags, masses, massBuffer_, massBufferMemory_);
	Vulkan::BufferUtil::CreateDeviceBuffer(commandPool, "Velocity", flags, speeds, speedBuffer_, speedBufferMemory_);
}

Scene::~Scene()
{
	speedBuffer_.reset();
	speedBufferMemory_.reset();
	massBuffer_.reset();
	massBufferMemory_.reset();
	positionBuffer_.reset();
	positionBufferMemory_.reset();
	indexBuffer_.reset();
	indexBufferMemory_.reset();
	vertexBuffer_.reset();
	vertexBufferMemory_.reset();
}

void Scene::updatePositions(double deltaTime)
{
	#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < positions.size(); ++i)
	{
		//glm::vec4 v = velocities[i];
		for (size_t j = 0; j < positions.size(); ++j)
		{
			if (i == j) continue;
			float gravConstant = 6.67e-11;
			glm::vec4 dir = positions[j] - positions[i];
			velocities[i] += 100000*(float)deltaTime * masses[j] * gravConstant * glm::normalize(dir) / glm::sqrt(glm::dot(dir, dir));
		}
		speeds[i] = glm::sqrt(glm::dot(velocities[i], velocities[i]));
		positions[i] += velocities[i] * (float)deltaTime;

	}
}

void Scene::updateBuffer(Vulkan::CommandPool& commandPool)
{
	Vulkan::BufferUtil::CopyFromStagingBuffer(commandPool, *positionBuffer_, positions);
	Vulkan::BufferUtil::CopyFromStagingBuffer(commandPool, *speedBuffer_, speeds);
}
