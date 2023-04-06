#pragma once

#include "../Vulkan/PipelineLayout.hpp"
#include "../Vulkan/Device.hpp"
#include "../Vulkan/RenderPass.hpp"
#include "../Vulkan/DescriptorSetLayout.hpp"
#include "../Vulkan/DescriptorSetManager.hpp"
#include "../Vulkan/SwapChain.hpp"
#include "../Vulkan/CommandPool.hpp"
#include "../Vulkan/Buffer.hpp"
#include "../Vulkan/DeviceMemory.hpp"
#include "../PathTracer/Camera.hpp"
#include "../PathTracer/Scene.hpp"

#include <memory>

namespace Vulkan
{
	class ComputeTracer
	{
	public:
		ComputeTracer(const Vulkan::Device& device, Vulkan::CommandPool& commandPool, VkDescriptorImageInfo& imageInfo, uint32_t imgWidth, uint32_t imgHeight);
		~ComputeTracer();

		void resizeComputeTarget(uint32_t imgWidth, uint32_t imgHeight, VkDescriptorImageInfo& imageDescriptor);
		void bindPipeline(VkCommandBuffer& commandBuffer)
		{
			vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
		}
		void updateCameraUBO()
		{
			camera_.updateCameraUBO();
		}

		VkDescriptorSet ComputeTextureDescriptorSet() const;
		const class PipelineLayout& PipelineLayout() const { return *pipelineLayout_; }
	private:
		void createAccumulatorImage(uint32_t imgWidth, uint32_t imgHeight);
		void deleteAccumulatorImage();

		const Device& device_;
		CommandPool& commandPool_;
		Camera camera_;

		VULKAN_HANDLE(VkPipeline, pipeline_)

		std::unique_ptr<DescriptorSetManager> descriptorSetManager_;
		std::unique_ptr<Vulkan::PipelineLayout> pipelineLayout_;

		std::unique_ptr<class Image> accumulatorImage_;
		std::unique_ptr<class DeviceMemory> accumulatorImageMemory_;
		std::unique_ptr<class ImageView> accumulatorImageView_;
		std::unique_ptr<class Sampler> accumulatorImageSampler_;
		VkDescriptorImageInfo accumulatorImageDescriptorInfo_;
		// storage buffers for all the yummy scene data
		std::unique_ptr<Buffer> vertexBuffer_;
		std::unique_ptr<DeviceMemory> vertexBufferMemory_;

		std::unique_ptr<Buffer> normalBuffer_;
		std::unique_ptr<DeviceMemory> normalBufferMemory_;

		std::unique_ptr<Buffer> indexBuffer_;
		std::unique_ptr<DeviceMemory> indexBufferMemory_;

		std::unique_ptr<Buffer> triangleBuffer_;
		std::unique_ptr<DeviceMemory> triangleBufferMemory_;

		std::unique_ptr<Buffer> bvhNodeBuffer_;
		std::unique_ptr<DeviceMemory> bvhNodeBufferMemory_;

		std::unique_ptr<Buffer> materialBuffer_;
		std::unique_ptr<DeviceMemory> materialBufferMemory_;
		const Scene scene;

	};
}