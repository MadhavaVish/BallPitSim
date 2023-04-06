#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <string>
#include <functional>

namespace Vulkan
{
	struct Transform {
		Transform(glm::mat4 transform) : objToWorld(transform)
		{
			worldToObj = glm::inverse(objToWorld);
		};
		glm::mat4 objToWorld;
		glm::mat4 worldToObj;
	};

	struct AABB
	{
		glm::vec3 bmin{ std::numeric_limits<float>::infinity() }, bmax{ -std::numeric_limits<float>::infinity() };
		void grow(glm::vec3 p) { bmin = glm::min(bmin, p); bmax = glm::max(bmax, p); }
		void grow(AABB& b) { if (b.bmin.x != std::numeric_limits<float>::infinity()) { grow(b.bmin); grow(b.bmax); } }
		float area()
		{
			glm::vec3 e = bmax - bmin; // box extent
			return e.x * e.y + e.y * e.z + e.z * e.x;
		}
	};

	template <typename T, typename... Rest>
	void hashCombine(std::size_t& seed, const T& v, const Rest&... rest) {
		seed ^= std::hash<T>{}(v)+0x9e3779b9 + (seed << 6) + (seed >> 2);
		(hashCombine(seed, rest), ...);
	};

	struct alignas(16) Tri
	{
		Tri(uint32_t offset, uint32_t firstIndex, uint32_t materialIndex, uint32_t smooth)
			: modelOffset(offset), v_indices(firstIndex), materialIdx(materialIndex), shadeSmooth(smooth)
		{};
		uint32_t shadeSmooth;
		uint32_t materialIdx;
		uint32_t modelOffset;
		uint32_t v_indices;
	};

	struct TriangleBVHData
	{
		glm::vec3 centroid;
		AABB triBound;
	};

	struct Vertex {
		glm::vec3 position{};
		glm::vec3 normal{};
		glm::vec2 uv{};
		bool operator==(const Vertex& other) const {
			return position == other.position && normal == other.normal && uv == other.uv;
		}
	};
	struct Model
	{
		Model(
			const std::string& filepath, 
			const Transform& transform, 
			const uint32_t materialIdx, 
			std::vector<glm::vec4>& vertices,
			std::vector<glm::vec4>& normals,
			std::vector<uint32_t>& indices,
			std::vector<Tri>& triangles,
			std::vector<TriangleBVHData>& triboundsinfo);

	};

	
}