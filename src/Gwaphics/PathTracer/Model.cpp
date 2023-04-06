#include "Model.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>
#include <unordered_map>
#include <iostream>

namespace std
{
    template <>
    struct hash<Vulkan::Vertex> {
        size_t operator()(Vulkan::Vertex const& vertex) const {
            size_t seed = 0;
            Vulkan::hashCombine(seed, vertex.position, vertex.normal, vertex.uv);
            return seed;
        }
    };
}
namespace Vulkan
{
    Model::Model(
        const std::string& filepath, 
        const Transform& transform, 
        const uint32_t materialIdx, 
        std::vector<glm::vec4>& vertices, 
        std::vector<glm::vec4>& normals, 
        std::vector<uint32_t>& indices, 
        std::vector<Tri>& triangles, 
        std::vector<TriangleBVHData>& triboundsinfo)
    {
        size_t offset = vertices.size();
        size_t offsetind = indices.size();
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath.c_str()))
        {
            std::cout << "pain" << std::endl;
            throw std::runtime_error(warn + err);
        }
        std::unordered_map<Vertex, uint32_t> uniqueVertices;
        glm::mat4 inverseTranspose = glm::transpose(transform.worldToObj);
        for (const auto& shape : shapes)
        {
            for (const auto& index : shape.mesh.indices)
            {
                Vertex vertex{};

                if (index.vertex_index >= 0) {
                    vertex.position = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2],
                    };
                }

                if (index.normal_index >= 0) {
                    vertex.normal = {
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2],
                    };
                }

                if (index.texcoord_index >= 0) {
                    vertex.uv = {
                        attrib.texcoords[2 * index.texcoord_index + 0],
                        attrib.texcoords[2 * index.texcoord_index + 1],
                    };
                }
                if (uniqueVertices.count(vertex) == 0) {
                    uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size() - offset);
                    glm::vec4 position(glm::vec3(transform.objToWorld * glm::vec4(vertex.position, 1.f)), vertex.uv.x);
                    vertices.push_back(position);
                    glm::vec4 normal(glm::normalize(glm::vec3(inverseTranspose * glm::vec4(vertex.normal, 0.f))), vertex.uv.y);
                    normals.push_back(normal);
                }
                indices.push_back(uniqueVertices[vertex]);
            }
        }

        for (size_t i = offsetind; i < indices.size(); i += 3)
        {
            glm::vec3 v0 = vertices[offset + indices[i]], v1 = vertices[offset + indices[i+1]], v2 = vertices[offset + indices[i+2]];
            
            TriangleBVHData data;
            data.centroid = (v0 + v1 + v2) / 3.f;
            data.triBound.grow(v0);
            data.triBound.grow(v1);
            data.triBound.grow(v2);
            triboundsinfo.push_back(data);

            Tri tri(static_cast<uint32_t>(offset), static_cast<uint32_t>(i), materialIdx, true);
            triangles.push_back(tri);
        }
    }
}