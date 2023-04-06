#pragma once

#include "../Vulkan/PipelineLayout.hpp"
#include "../Vulkan/Device.hpp"
#include "../Vulkan/RenderPass.hpp"
#include "../Vulkan/DescriptorSetLayout.hpp"
#include "../Vulkan/DescriptorSetManager.hpp"
#include "../Vulkan/SwapChain.hpp"
#include "UniformBuffer.hpp"
#include <memory>

namespace Vulkan
{

	struct PipelineConfigInfo {
		PipelineConfigInfo() = default;
		PipelineConfigInfo(const PipelineConfigInfo&) = delete;
		PipelineConfigInfo& operator=(const PipelineConfigInfo&) = delete;

		std::vector<VkVertexInputBindingDescription> bindingDescriptions{};
		std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
		VkPipelineViewportStateCreateInfo viewportInfo;
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyInfo;
		VkPipelineRasterizationStateCreateInfo rasterizationInfo;
		VkPipelineMultisampleStateCreateInfo multisampleInfo;
		VkPipelineColorBlendAttachmentState colorBlendAttachment;
		VkPipelineColorBlendStateCreateInfo colorBlendInfo;
		VkPipelineDepthStencilStateCreateInfo depthStencilInfo;
		std::vector<VkDynamicState> dynamicStateEnables;
		VkPipelineDynamicStateCreateInfo dynamicStateInfo;
		VkPipelineLayout pipelineLayout = nullptr;
		VkRenderPass renderPass = nullptr;
		uint32_t subpass = 0;
	};

	class SimpleQuadPipeline
	{
	public:
		SimpleQuadPipeline(const Vulkan::SwapChain& swapChain, const Vulkan::RenderPass& renderPass, const std::vector<Vulkan::UniformBuffer>& uniformBuffers, VkDescriptorImageInfo& imageDescriptor);
		~SimpleQuadPipeline();

		void updateQuadTextureDescriptor(const std::vector<Vulkan::UniformBuffer>& uniformBuffers, VkDescriptorImageInfo& imageDescriptor);

		void bindPipeline(VkCommandBuffer& commandBuffer)
		{
			vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
		}

		void renderQuad(VkCommandBuffer& commandBuffer)
		{
			vkCmdDraw(commandBuffer, 6, 1, 0, 0);
		}

		VkDescriptorSet DescriptorSet(uint32_t index) const;
		//VkDescriptorSet QuadTextureDescriptorSet() const;
		const class PipelineLayout& PipelineLayout() const { return *pipelineLayout_; }

	private:
		const SwapChain& swapChain_;

		VULKAN_HANDLE(VkPipeline, pipeline_)

		std::unique_ptr<DescriptorSetManager> descriptorSetManager_;
		//std::unique_ptr<DescriptorSetManager> imageDescriptorSetManager_;
		std::unique_ptr<Vulkan::PipelineLayout> pipelineLayout_;
	};
}