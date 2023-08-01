#pragma once
#include "../Utilities/Model.hpp"
#include "Constraint.hpp"
#include "BallLoader.hpp"
#include "Ball.hpp"
#include "../Vulkan/Buffer.hpp"
#include "../Vulkan/BufferUtil.hpp"
#include "../Vulkan/DeviceMemory.hpp"
#include <memory>
#include <Eigen/Dense>

using namespace Eigen;

class Scene
{
public:
	double currTime;
	double currStep;
	double physicsStepTime = 3e-2;
	double CRCoeff = 1.0;
	double dragCoeff = 0.0;
	double tolerance = 1e-3;
	int maxIterations = 1e5;
	bool isRunning = false;
	Scene(Vulkan::CommandPool& commandPool);
	~Scene();
	const Vulkan::Buffer& VertexBuffer() const { return *vertexBuffer_; }
	const Vulkan::Buffer& IndexBuffer() const { return *indexBuffer_; }
	const Vulkan::Buffer& PositionBuffer() const { return *positionBuffer_; }
	const Vulkan::Buffer& ColorBuffer() const { return *colorBuffer_; }
	const uint32_t NumVerts() const { return vertices.size(); }
	const uint32_t NumIndices() const { return indices.size(); }
	const uint32_t NumParticles() const { return positions.size(); }
	void updatePositions(double deltaTime);
	void updateBuffer(Vulkan::CommandPool& commandPool);
	
	void handleCollision(Constraint c); // int b1, int b2, const double& depth, const RowVector3d& contactNormal, const RowVector3d& penPosition, const double CRCoeff, const double tolerance);
	void addBunny();
	void addPool();
	void addFloor();
	void updateScene(const double timeStep, const double CRCoeff, const double dragCoeff, const double tolerance, const int maxIterations);

private:
	std::vector<Vertex> vertices;
	std::unique_ptr<Vulkan::Buffer> vertexBuffer_;
	std::unique_ptr<Vulkan::DeviceMemory> vertexBufferMemory_;

	std::vector<uint32_t> indices;
	std::unique_ptr<Vulkan::Buffer> indexBuffer_;
	std::unique_ptr<Vulkan::DeviceMemory> indexBufferMemory_;

	uint32_t numParticles;
	std::vector<glm::vec4> positions;
	std::unique_ptr<Vulkan::Buffer> positionBuffer_;
	std::unique_ptr<Vulkan::DeviceMemory> positionBufferMemory_;
	std::vector<glm::vec4> velocities;

	std::vector<Ball> balls;
	std::vector<glm::vec4> ballsColor;
	std::unique_ptr<Vulkan::Buffer> colorBuffer_;
	std::unique_ptr<Vulkan::DeviceMemory> colorBufferMemory_;
	std::vector<Mesh> meshes;
	std::vector<Constraint> constraints;
};