#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <functional>
#include "../Vulkan/Vulkan.hpp"


struct Transform {
	Transform(glm::mat4 transform) : objToWorld(transform)
	{
		worldToObj = glm::inverse(objToWorld);
	};
	glm::mat4 objToWorld;
	glm::mat4 worldToObj;
};

template <typename T, typename... Rest>
void hashCombine(std::size_t& seed, const T& v, const Rest&... rest) {
	seed ^= std::hash<T>{}(v)+0x9e3779b9 + (seed << 6) + (seed >> 2);
	(hashCombine(seed, rest), ...);
};

struct Vertex {
	glm::vec3 position{};
	glm::vec3 normal{};
	bool operator==(const Vertex& other) const {
		return position == other.position && normal == other.normal;
	}
	static VkVertexInputBindingDescription GetBindingDescription()
	{
		VkVertexInputBindingDescription bindingDescription = {};
		bindingDescription.binding = 0;
		bindingDescription.stride = sizeof(Vertex);
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
		return bindingDescription;
	}
	static std::array<VkVertexInputAttributeDescription, 2> GetAttributeDescriptions()
	{
		std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {};

		attributeDescriptions[0].binding = 0;
		attributeDescriptions[0].location = 0;
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[0].offset = offsetof(Vertex, position);

		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(Vertex, normal);

		return attributeDescriptions;
	}
};

struct Model
{
	Model(
		const std::string& filepath, 
		std::vector<Vertex>& vertices,
		std::vector<uint32_t>& indices);
};