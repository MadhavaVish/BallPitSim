#pragma once
#include "../Utilities/Model.hpp"
#include "../Vulkan/Buffer.hpp"
#include "../Vulkan/BufferUtil.hpp"
#include "../Vulkan/DeviceMemory.hpp"
#include <memory>
class Scene
{
public:
	Scene(Vulkan::CommandPool& commandPool);
	~Scene();
	const Vulkan::Buffer& VertexBuffer() const { return *vertexBuffer_; }
	const Vulkan::Buffer& IndexBuffer() const { return *indexBuffer_; }
	const Vulkan::Buffer& PositionBuffer() const { return *positionBuffer_; }
	const uint32_t NumVerts() const { return vertices.size(); }
	const uint32_t NumIndices() const { return indices.size(); }
	const uint32_t NumParticles() const { return numParticles; }
	void updatePositions(double deltaTime);
	void updateBuffer(Vulkan::CommandPool& commandPool);
	
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
};