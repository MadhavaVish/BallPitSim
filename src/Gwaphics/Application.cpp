#include "Application.hpp"
#include "Vulkan/Buffer.hpp"
#include "Vulkan/CommandPool.hpp"
#include "Vulkan/CommandBuffers.hpp"
#include "Vulkan/DebugUtilsMessenger.hpp"
#include "Vulkan/DepthBuffer.hpp"
#include "Vulkan/Device.hpp"
#include "Vulkan/Fence.hpp"
#include "Vulkan/FrameBuffer.hpp"
#include "Vulkan/Instance.hpp"
#include "Vulkan/PipelineLayout.hpp"
#include "Vulkan/RenderPass.hpp"
#include "Vulkan/Semaphore.hpp"
#include "Vulkan/Surface.hpp"
#include "Vulkan/SwapChain.hpp"
#include "Vulkan/Window.hpp"
#include "Vulkan/Image.hpp"
#include "Vulkan/ImageView.hpp"
#include "Vulkan/Sampler.hpp"
#include "Vulkan/ImageMemoryBarrier.hpp"
#include "Gwaphics/Pipelines/SimpleQuadPipeline.hpp"
#include "ImGui/backends/imgui_impl_vulkan.h"

#include <stdexcept>
#include <array>
#include <iostream>
#include <glm/gtx/string_cast.hpp>
namespace Vulkan {

Application::Application(const WindowConfig& windowConfig, const VkPresentModeKHR presentMode, const bool enableValidationLayers) :
	presentMode_(presentMode)
{
	const auto validationLayers = enableValidationLayers
		? std::vector<const char*>{"VK_LAYER_KHRONOS_validation"}
		: std::vector<const char*>();

	window_.reset(new class Window(windowConfig));
	instance_.reset(new Instance(*window_, validationLayers, VK_API_VERSION_1_2));
	debugUtilsMessenger_.reset(enableValidationLayers ? new DebugUtilsMessenger(*instance_, VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT) : nullptr);
	surface_.reset(new Surface(*instance_));
	settings.particleScale = &particleScale;
	modelViewController_.Reset(glm::translate(glm::mat4(1), glm::vec3(0, 0, -2)));
}

Application::~Application()
{
	Application::DeleteSwapChain();
	scene_.reset();
	commandPool_.reset();
	device_.reset();
	surface_.reset();
	debugUtilsMessenger_.reset();
	instance_.reset();
	window_.reset();
}

const std::vector<VkExtensionProperties>& Application::Extensions() const
{
	return instance_->Extensions();
}

const std::vector<VkLayerProperties>& Application::Layers() const
{
	return instance_->Layers();
}

const std::vector<VkPhysicalDevice>& Application::PhysicalDevices() const
{
	return instance_->PhysicalDevices();
}

void Application::SetPhysicalDevice(VkPhysicalDevice physicalDevice)
{
	if (device_)
	{
		throw(std::logic_error("physical device has already been set"));
	}

	std::vector<const char*> requiredExtensions = 
	{
		// VK_KHR_swapchain
		VK_KHR_SWAPCHAIN_EXTENSION_NAME
	};

	VkPhysicalDeviceFeatures deviceFeatures = {};
	
	SetPhysicalDevice(physicalDevice, requiredExtensions, deviceFeatures, nullptr);
	OnDeviceSet();

	// Create swap chain and command buffers.
	CreateSwapChain();
}

void Application::Run()
{
	if (!device_)
	{
		throw(std::logic_error("physical device has not been set"));
	}

	currentFrame_ = 0;

	window_->DrawFrame = [this]() { DrawFrame(); };
	window_->OnKey = [this](const int key, const int scancode, const int action, const int mods) { OnKey(key, scancode, action, mods); };
	window_->OnCursorPosition = [this](const double xpos, const double ypos) { OnCursorPosition(xpos, ypos); };
	window_->OnMouseButton = [this](const int button, const int action, const int mods) { OnMouseButton(button, action, mods); };
	window_->OnScroll = [this](const double xoffset, const double yoffset) { OnScroll(xoffset, yoffset); };
	window_->Run();
	device_->WaitIdle();
}

void Application::SetPhysicalDevice(
	VkPhysicalDevice physicalDevice, 
	std::vector<const char*>& requiredExtensions, 
	VkPhysicalDeviceFeatures& deviceFeatures,
	void* nextDeviceFeatures)
{
	device_.reset(new class Device(physicalDevice, *surface_, requiredExtensions, deviceFeatures, nextDeviceFeatures));
	commandPool_.reset(new class CommandPool(*device_, device_->GraphicsFamilyIndex(), true));
	scene_.reset(new Scene(CommandPool()));
}

void Application::OnDeviceSet()
{
}

void Application::CreateSwapChain()
{
	// Wait until the window is visible.
	while (window_->IsMinimized())
	{
		window_->WaitForEvents();
	}

	swapChain_.reset(new class SwapChain(*device_, presentMode_));
	depthBuffer_.reset(new class DepthBuffer(*commandPool_, swapChain_->Extent()));

	for (size_t i = 0; i != swapChain_->ImageViews().size(); ++i)
	{
		imageAvailableSemaphores_.emplace_back(*device_);
		renderFinishedSemaphores_.emplace_back(*device_);
		inFlightFences_.emplace_back(*device_, true);
		quadUniformBuffers_.emplace_back(*device_);
	}

	renderPass_.reset(new class RenderPass(*swapChain_, *depthBuffer_));
	RenderPassConfig& conf = renderPass_->GetRenderPassConfig();
	renderPass_->BuildRenderPass();

	for (const auto& imageView : swapChain_->ImageViews())
	{
		swapChainFramebuffers_.emplace_back(*imageView, *renderPass_, swapChain_->Extent());
	}

	commandBuffers_.reset(new CommandBuffers(*commandPool_, static_cast<uint32_t>(swapChainFramebuffers_.size())));

	userInterface_.reset(new UserInterface(CommandPool(), SwapChain(), DepthBuffer(), RenderPass()));
}

void Application::DeleteSwapChain()
{
	quadPipeline_.reset();
	DeleteViewPort();
	dsetsInit = false;
	userInterface_.reset();
	commandBuffers_.reset();
	swapChainFramebuffers_.clear();
	renderPass_.reset();
	quadUniformBuffers_.clear();
	inFlightFences_.clear();
	renderFinishedSemaphores_.clear();
	imageAvailableSemaphores_.clear();
	depthBuffer_.reset();
	swapChain_.reset();
}

void Application::CreateViewport()
{
	viewportDepthBuffer_.reset(new class DepthBuffer(*commandPool_, viewportExtent_));
	viewportRenderPass_.reset(new class RenderPass(*swapChain_, *viewportDepthBuffer_));
	RenderPassConfig& conf = viewportRenderPass_->GetRenderPassConfig();
	conf.colorAttachment.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
	viewportRenderPass_->BuildRenderPass(); 

	SamplerConfig config{};
	for (uint32_t i = 0; i < swapChain_->Images().size(); i++)
	{
		viewportImages_.emplace_back(*device_,viewportExtent_, VK_FORMAT_B8G8R8A8_UNORM, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
		viewportImageMemory_.emplace_back(viewportImages_[i].AllocateMemory(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
		viewportImages_[i].TransitionImageLayout(*commandPool_, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		viewportImageViews_.push_back(std::make_unique<ImageView>(*device_, viewportImages_[i].Handle(), viewportImages_[i].Format(), VK_IMAGE_ASPECT_COLOR_BIT));
		viewportFramebuffers_.emplace_back(*viewportImageViews_[i], *viewportRenderPass_, viewportExtent_);
		viewportSamplers_.push_back(std::make_unique <Sampler>(*device_, config));
	}
	if (dsetsInit)
	{
		for (uint32_t i = 0; i < viewportImageViews_.size(); i++)
		{
			VkDescriptorImageInfo desc_image[1] = {};
			desc_image[0].sampler = (*viewportSamplers_[i]).Handle();
			desc_image[0].imageView = (*viewportImageViews_[i]).Handle();
			desc_image[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

			VkWriteDescriptorSet write_desc[1] = {};
			write_desc[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			write_desc[0].dstSet = viewportDsets_[i];
			write_desc[0].descriptorCount = 1;
			write_desc[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
			write_desc[0].pImageInfo = desc_image;
			vkUpdateDescriptorSets(device_->Handle(), 1, write_desc, 0, NULL);
		}
	}
	else
	{
		viewportDsets_.resize(viewportImageViews_.size());
		for (uint32_t i = 0; i < viewportImageViews_.size(); i++)
		{
			viewportDsets_[i]=(ImGui_ImplVulkan_AddTexture(viewportSamplers_[i]->Handle(), viewportImageViews_[i]->Handle(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));
		}
		dsetsInit = true;

	}
	viewportInit = true;

	quadPipeline_.reset(new SimpleQuadPipeline(SwapChain(), *viewportRenderPass_, quadUniformBuffers_, scene_->PositionBuffer(), scene_->ColorBuffer()));


}

void Application::DeleteViewPort()
{
	device_->WaitIdle();

	viewportFramebuffers_.clear();
	viewportDepthBuffer_.reset(); 
	viewportSamplers_.clear();
	viewportImageViews_.clear();
	viewportImages_.clear();
	viewportImageMemory_.clear();
	viewportRenderPass_.reset();
	viewportInit = false;
}

void Application::DrawFrame()
{
	constexpr auto noTimeout = std::numeric_limits<uint64_t>::max();

	auto& inFlightFence = inFlightFences_[currentFrame_];
	const auto imageAvailableSemaphore = imageAvailableSemaphores_[currentFrame_].Handle();
	const auto renderFinishedSemaphore = renderFinishedSemaphores_[currentFrame_].Handle();

	inFlightFence.Wait(noTimeout);

	uint32_t imageIndex;
	auto result = vkAcquireNextImageKHR(device_->Handle(), swapChain_->Handle(), noTimeout, imageAvailableSemaphore, nullptr, &imageIndex);

	if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
	{
		RecreateSwapChain();
		return;
	}

	if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
	{
		throw(std::runtime_error(std::string("failed to acquire next image (") + ToString(result) + ")"));
	}
	if (viewportExtent_.width != newViewportExtent_.width || viewportExtent_.height != newViewportExtent_.height)
	{
		viewportExtent_ = newViewportExtent_;
		DeleteViewPort();
		CreateViewport();
	}

	const auto commandBuffer = commandBuffers_->Begin(imageIndex);
	Render(commandBuffer, imageIndex);
	commandBuffers_->End(imageIndex);

	UpdateUniformBuffer(imageIndex);

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	VkCommandBuffer commandBuffers[]{ commandBuffer };
	VkSemaphore waitSemaphores[] = { imageAvailableSemaphore };
	VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	VkSemaphore signalSemaphores[] = { renderFinishedSemaphore };

	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = waitSemaphores;
	submitInfo.pWaitDstStageMask = waitStages;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = commandBuffers;
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = signalSemaphores;

	inFlightFence.Reset();

	Check(vkQueueSubmit(device_->GraphicsQueue(), 1, &submitInfo, inFlightFence.Handle()),
		"submit draw command buffer");

	VkSwapchainKHR swapChains[] = { swapChain_->Handle() };
	VkPresentInfoKHR presentInfo = {};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	presentInfo.waitSemaphoreCount = 1;
	presentInfo.pWaitSemaphores = signalSemaphores;
	presentInfo.swapchainCount = 1;
	presentInfo.pSwapchains = swapChains;
	presentInfo.pImageIndices = &imageIndex;
	presentInfo.pResults = nullptr; // Optional

	result = vkQueuePresentKHR(device_->PresentQueue(), &presentInfo);

	if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
	{
		RecreateSwapChain();
		return;
	}
	
	if (result != VK_SUCCESS)
	{
		throw(std::runtime_error(std::string("failed to present next image (") + ToString(result) + ")"));
	}
	currentFrame_ = (currentFrame_ + 1) % inFlightFences_.size();
}

void Application::Render(VkCommandBuffer commandBuffer, const uint32_t imageIndex)
{
	const auto prevTime = time_;
	time_ = Window().GetTime();
	frameTime = time_ - prevTime;
	modelViewController_.UpdateCamera(2.f, frameTime);

	std::array<VkClearValue, 2> clearValues = {};
	clearValues[0].color = { {0.1f, 0.1f, 0.1f, 1.0f} };
	clearValues[1].depthStencil = { 1.0f, 0 };

	VkRenderPassBeginInfo renderPassInfo = {};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassInfo.renderArea.offset = { 0, 0 };
	renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());

	frameStats.initView = viewportInit;

	if (viewportInit)
	{
		renderPassInfo.renderPass = viewportRenderPass_->Handle();
		renderPassInfo.framebuffer = viewportFramebuffers_[imageIndex].Handle();
		renderPassInfo.pClearValues = clearValues.data();
		renderPassInfo.renderArea.extent = viewportExtent_;

		vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
		VkViewport viewport{};
		viewport.x = 0.0f;
		viewport.y = 0.0f;
		viewport.width = static_cast<float>(viewportExtent_.width);
		viewport.height = static_cast<float>(viewportExtent_.height);
		viewport.minDepth = 0.0f;
		viewport.maxDepth = 1.0f;
		VkRect2D scissor{ {0, 0}, viewportExtent_ };
		vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
		vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
		{
			VkDescriptorSet descriptorSets[] = { quadPipeline_->DescriptorSet(imageIndex) };
			VkBuffer vertexBuffers[] = { scene_->VertexBuffer().Handle() };
			const VkBuffer indexBuffer = scene_->IndexBuffer().Handle();
			VkDeviceSize offsets[] = { 0 };

			vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, quadPipeline_->Handle());
			vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, quadPipeline_->PipelineLayout().Handle(), 0, 1, descriptorSets, 0, nullptr);
			vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
			vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
			uint32_t vertexOffset = 0;
			uint32_t indexOffset = 0;
			const auto vertexCount = scene_->NumVerts();
			const auto indexCount = scene_->NumIndices();
			vkCmdDrawIndexed(commandBuffer, indexCount, scene_->NumParticles(), 0, 0, 0);
		}
		vkCmdEndRenderPass(commandBuffer);
		frameStats.viewImage = &viewportDsets_[imageIndex];
	}

	newViewportExtent_ = userInterface_->DrawUI(frameStats, settings);

	clearValues[0].color = { {0.0f, 0.0f, 0.0f, 1.0f} };

	renderPassInfo.renderPass = renderPass_->Handle();
	renderPassInfo.framebuffer = swapChainFramebuffers_[imageIndex].Handle();
	renderPassInfo.pClearValues = clearValues.data();
	renderPassInfo.renderArea.extent = swapChain_->Extent();
	
	vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
	{
		userInterface_->Render(commandBuffer);
	}
	vkCmdEndRenderPass(commandBuffer);
}

void Application::OnKey(int key, int scancode, int action, int mods)
{
	

	// Camera motions
	
	modelViewController_.OnKey(key, scancode, action, mods);
}

void Application::OnCursorPosition(double xpos, double ypos)
{
	if (!HasSwapChain())
	{
		return;
	}

	// Camera motions
	modelViewController_.OnCursorPosition(xpos, ypos);
}

void Application::OnMouseButton(int button, int action, int mods)
{
	if (!HasSwapChain())
	{
		return;
	}
	modelViewController_.OnMouseButton(button, action, mods);
}

void Application::OnScroll(double xoffset, double yoffset)
{
}

void Application::UpdateUniformBuffer(const uint32_t imageIndex)
{
	scene_->updatePositions(frameTime);
	
	UniformBufferObject ubo = {};
	ubo.ModelView = modelViewController_.ModelView();
	ubo.Projection = glm::perspective(glm::radians(45.f), viewportExtent_.width / static_cast<float>(viewportExtent_.height), 0.1f, 10000.0f);
	ubo.Particlescale = particleScale;
	quadUniformBuffers_[imageIndex].SetValue(ubo);
	scene_->updateBuffer(*commandPool_);
}

void Application::RecreateSwapChain()
{
	device_->WaitIdle();
	DeleteSwapChain();
	CreateSwapChain();
	CreateViewport();
}

}
