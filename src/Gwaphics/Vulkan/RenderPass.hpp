#pragma once

#include "Vulkan.hpp"
#include <vector>

namespace Vulkan
{
	class DepthBuffer;
	class SwapChain;

	struct RenderPassConfig final
	{
		RenderPassConfig(VkFormat colorAttachmentFormat, VkFormat depthAttachmentFormat)
		{
			colorAttachment.format = colorAttachmentFormat;
			colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
			colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
			colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

			depthAttachment.format = depthAttachmentFormat;
			depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
			depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
			depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

			colorAttachmentRef.attachment = 0;
			colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

			depthAttachmentRef.attachment = 1;
			depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

			VkSubpassDescription subpass = {};
			subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
			subpass.colorAttachmentCount = 1;
			subpass.pColorAttachments = &colorAttachmentRef;
			subpass.pDepthStencilAttachment = &depthAttachmentRef;
			subpasses.push_back(subpass);

			VkSubpassDependency dependency = {};
			dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
			dependency.dstSubpass = 0;
			dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
			dependency.srcAccessMask = 0;
			dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
			dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			dependencies.push_back(dependency);
		}

		VkAttachmentDescription colorAttachment = {};
		VkAttachmentDescription depthAttachment = {};
		VkAttachmentReference colorAttachmentRef = {};
		VkAttachmentReference depthAttachmentRef = {};
		std::vector<VkSubpassDescription> subpasses{};
		std::vector<VkSubpassDependency> dependencies{};
	};

	class RenderPass final
	{
	public:

		VULKAN_NON_COPIABLE(RenderPass)

		RenderPass(const SwapChain& swapChain, const DepthBuffer& depthBuffer);
		~RenderPass();
		const void BuildRenderPass();
		RenderPassConfig& GetRenderPassConfig() { return renderpassConfig_; };
		const class SwapChain& SwapChain() const { return swapChain_; }
		const class DepthBuffer& DepthBuffer() const { return depthBuffer_; }

	private:

		const class SwapChain& swapChain_;
		const class DepthBuffer& depthBuffer_;
		RenderPassConfig renderpassConfig_;
		VULKAN_HANDLE(VkRenderPass, renderPass_)
	};

}
