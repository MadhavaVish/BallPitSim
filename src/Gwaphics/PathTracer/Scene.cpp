#include "Scene.hpp"
#include <chrono>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

namespace Vulkan
{
	typedef std::chrono::high_resolution_clock Clock;
	Scene::Scene()
	{
		AddMaterial({ 0.7, 0.34, 0.21 }, 1.f);
		//addModel("assets/bunny.obj", glm::mat4(2.f), 0);
		AddMaterial({ 0.156863f, 0.803922f, 0.172549f }, 0.f);
		AddMaterial({ 0.803922f, 0.152941f, 0.152941f }, 0.f);
		AddMaterial({ 0.803922f, 0.803922f, 0.803922f }, 0.f);
		AddMaterial({ 0.81, 0.35, 0.07 }, 0.f);
		addModel("assets/models/Cornell/Left.obj", glm::mat4(1.f), 1);
		addModel("assets/models/Cornell/Right.obj", glm::mat4(1.f), 2);
		glm::mat4 transform(4.f);
		transform = glm::translate(transform, { 0, -0.8, 0 });
		addModel("assets/models/bunny.obj", transform, 4);
		addModel("assets/models/Cornell/Bottom.obj", glm::mat4(1.f), 3);
		addModel("assets/models/Cornell/Back.obj", glm::mat4(1.f), 3);

		for (unsigned int i = 0; i < triangles.size(); i++)
		{
			triIdx.push_back(i);
		}
		BuildBVH();

		std::vector<Tri> sortedTris;
		sortedTris.reserve(triangles.size());
		for (auto i = 0; i < triIdx.size(); i++)
		{
			sortedTris.push_back(triangles[triIdx[i]]);
		}
		triangles = sortedTris;
	}

	void Scene::AddMaterial(const glm::vec3 albedo, const float& radiance)
	{
		Material mat;
		mat.albedo = glm::vec4(albedo, radiance);
		materials.push_back(mat);
	}

	void Scene::addModel(const std::string& filepath, Transform transform, uint32_t material)
	{
		Model(filepath, transform, material, vertices, normals, indices, triangles, triboundsinfo);
	}

	static int BVHDepth = 0;
	void Scene::BuildBVH()
	{
		unsigned int N = static_cast<unsigned int>(triangles.size());
		// create the BVH node pool
		//bvhNode = (BVHNode*)_aligned_malloc(sizeof(BVHNode) * N * 2, 64);
		bvhNode.resize(N * 2);
		// populate triangle index array
		//for (unsigned int i = 0; i < N; i++) triIdx[i] = i;
		// assign all triangles to root node
		BVHNode& root = bvhNode[rootNodeIdx];
		root.leftFirst = 0, root.triCount = N;
		UpdateNodeBounds(rootNodeIdx);
		// subdivide recursively
		auto t1 = Clock::now();
		Subdivide(rootNodeIdx, 0);
		auto t2 = Clock::now();
		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
		printf("BVH (%i nodes) constructed in %.8fms.\n", nodesUsed, time_span.count() * 1000);
		std::cout << "BVH Depth: " << BVHDepth << std::endl;
	}

	void Scene::UpdateNodeBounds(unsigned int nodeIdx)
	{
		BVHNode& node = bvhNode[nodeIdx];
		AABB nodeBound;
		for (unsigned int first = node.leftFirst, i = 0; i < node.triCount; i++)
		{
			unsigned int leafTriIdx = triIdx[first + i];
			TriangleBVHData& leafTri = triboundsinfo[leafTriIdx];
			nodeBound.grow(leafTri.triBound);
		}
		node.minx = nodeBound.bmin.x;
		node.miny = nodeBound.bmin.y;
		node.minz = nodeBound.bmin.z;
		//node.aabbMin = nodeBound.bmin;
		node.maxx = nodeBound.bmax.x;
		node.maxy = nodeBound.bmax.y;
		node.maxz = nodeBound.bmax.z;
		//node.aabbMax = nodeBound.bmax;
	}

	static const int BINS = 8;
	float Scene::FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos)
	{
		float bestCost = std::numeric_limits<float>::infinity();
		for (int a = 0; a < 3; a++)
		{
			float boundsMin = std::numeric_limits<float>::infinity(), boundsMax = -std::numeric_limits<float>::infinity();
			for (unsigned int i = 0; i < node.triCount; i++)
			{
				TriangleBVHData& tri = triboundsinfo[triIdx[node.leftFirst + i]];
				glm::vec3 cent = tri.centroid;
				boundsMin = std::min(boundsMin, cent[a]);
				boundsMax = std::max(boundsMax, cent[a]);
			}
			if (boundsMin == boundsMax) continue;
			// populate the bins
			Bin bin[BINS];
			float scale = BINS / (boundsMax - boundsMin);
			for (unsigned int i = 0; i < node.triCount; i++)
			{
				TriangleBVHData& tri = triboundsinfo[triIdx[node.leftFirst + i]];
				glm::vec3 cent = tri.centroid;
				int binIdx = std::min(BINS - 1, (int)((cent[a] - boundsMin) * scale));
				bin[binIdx].triCount++;
				bin[binIdx].bounds.grow(tri.triBound);
			}
			// gather data for the 7 planes between the 8 bins
			float leftArea[BINS - 1], rightArea[BINS - 1];
			int leftCount[BINS - 1], rightCount[BINS - 1];
			AABB leftBox, rightBox;
			int leftSum = 0, rightSum = 0;
			for (int i = 0; i < BINS - 1; i++)
			{
				leftSum += bin[i].triCount;
				leftCount[i] = leftSum;
				leftBox.grow(bin[i].bounds);
				leftArea[i] = leftBox.area();
				rightSum += bin[BINS - 1 - i].triCount;
				rightCount[BINS - 2 - i] = rightSum;
				rightBox.grow(bin[BINS - 1 - i].bounds);
				rightArea[BINS - 2 - i] = rightBox.area();
			}
			// calculate SAH cost for the 7 planes
			scale = (boundsMax - boundsMin) / BINS;
			for (int i = 0; i < BINS - 1; i++)
			{
				float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
				if (planeCost < bestCost)
					axis = a, splitPos = boundsMin + scale * (i + 1), bestCost = planeCost;
			}
		}
		return bestCost;
	}

	float Scene::CalculateNodeCost(BVHNode& node)
	{
		float ex = node.maxx - node.minx; // extent of the node
		float ey = node.maxy - node.miny;
		float ez = node.maxz - node.minz;
		float surfaceArea = ex * ey + ey * ez + ez * ex;
		return node.triCount * surfaceArea;
	}

	void Scene::Subdivide(unsigned int nodeIdx, int depth)
	{
		// terminate recursion
		BVHNode& node = bvhNode[nodeIdx];
		// determine split axis using SAHw
		int axis;
		float splitPos;
		float splitCost = FindBestSplitPlane(node, axis, splitPos);
		float nosplitCost = CalculateNodeCost(node);
		if (splitCost >= nosplitCost) return;
		if (depth > BVHDepth) BVHDepth = depth;
		// in-place partition
		int i = node.leftFirst;
		int j = i + node.triCount - 1;
		while (i <= j)
		{
			TriangleBVHData& tri = triboundsinfo[triIdx[i]];
			glm::vec3 cent = tri.centroid;

			if (cent[axis] < splitPos)
				i++;
			else
				std::swap(triIdx[i], triIdx[j--]);
		}
		// abort split if one of the sides is empty
		int leftCount = i - node.leftFirst;
		if (leftCount == 0 || leftCount == node.triCount) return;
		// create child nodes
		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		bvhNode[leftChildIdx].leftFirst = node.leftFirst;
		bvhNode[leftChildIdx].triCount = leftCount;
		bvhNode[rightChildIdx].leftFirst = i;
		bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
		node.leftFirst = leftChildIdx;
		node.triCount = 0;
		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);
		// recurse
		Subdivide(leftChildIdx, depth + 1);
		Subdivide(rightChildIdx, depth + 1);
	}


}