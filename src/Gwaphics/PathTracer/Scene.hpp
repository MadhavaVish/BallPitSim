#pragma once
#include <glm/glm.hpp>

#include <string>
#include <vector>
#include "Model.hpp"

namespace Vulkan
{
	struct Material
	{
		glm::vec4 albedo;
	};
	
	class Scene 
	{
	private:
		struct BVHNode
		{
			float minx, miny, minz;
			uint32_t leftFirst;
			float maxx, maxy, maxz;
			uint32_t triCount;
		};

		struct Bin { AABB bounds; int triCount = 0; };
	public:
		Scene();

		void AddMaterial(const glm::vec3 albedo, const float& radiance);
		void addModel(const std::string& filepath, Transform transform, uint32_t material);
	private:
		void BuildBVH();
		void Subdivide(unsigned int nodeIdx, int depth);
		void UpdateNodeBounds(unsigned int nodeIdx);
		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);
		float CalculateNodeCost(BVHNode& node);
	public:
		std::vector<glm::vec4> vertices;
		std::vector<glm::vec4> normals;
		std::vector<uint32_t> indices;
		std::vector<Tri> triangles;
		std::vector<BVHNode> bvhNode;
		std::vector<Material> materials;
		//std::vector<Texture*> textures;
		std::vector<TriangleBVHData> triboundsinfo;
	private:
		std::vector<unsigned int> triIdx;
		unsigned int rootNodeIdx = 0, nodesUsed = 2;
	};

}