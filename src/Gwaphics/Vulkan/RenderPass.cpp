#include "RenderPass.hpp"
#include "DepthBuffer.hpp"
#include "Device.hpp"
#include "SwapChain.hpp"
#include <array>

namespace Vulkan {

RenderPass::RenderPass(
	const class SwapChain& swapChain, 
	const class DepthBuffer& depthBuffer) :
	swapChain_(swapChain),
	depthBuffer_(depthBuffer),
	renderpassConfig_(swapChain.Format(), depthBuffer.Format())
{
	
}

RenderPass::~RenderPass()
{
	if (renderPass_ != nullptr)
	{
		vkDestroyRenderPass(swapChain_.Device().Handle(), renderPass_, nullptr);
		renderPass_ = nullptr;
	}
}

const void RenderPass::BuildRenderPass()
{
	std::array<VkAttachmentDescription, 2> attachments =
	{
		renderpassConfig_.colorAttachment,
		renderpassConfig_.depthAttachment
	};

	VkRenderPassCreateInfo renderPassInfo = {};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassInfo.pAttachments = attachments.data();
	renderPassInfo.subpassCount = static_cast<uint32_t>(renderpassConfig_.subpasses.size());
	renderPassInfo.pSubpasses = renderpassConfig_.subpasses.data();
	renderPassInfo.dependencyCount = static_cast<uint32_t>(renderpassConfig_.dependencies.size());
	renderPassInfo.pDependencies = renderpassConfig_.dependencies.data();

	Check(vkCreateRenderPass(swapChain_.Device().Handle(), &renderPassInfo, nullptr, &renderPass_),
		"create render pass");
	return void();
}

}
