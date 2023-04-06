#pragma once
#include "Gwaphics/Vulkan/Vulkan.hpp"
#include <memory>

namespace Vulkan
{
	class CommandPool;
	class DepthBuffer;
	class DescriptorPool;
	class FrameBuffer;
	class RenderPass;
	class SwapChain;
}

struct UserSettings final
{
	// Scene
	int* ImageWidth;
	int* ImageHeight;
	float* Vignette;
	// Renderer
	bool AccumulateRays;

	// Camera

	// UI
};

struct Statistics final
{
	Statistics()
	{
		initView = false;
		viewImage = nullptr;
	}
	bool initView;
	VkDescriptorSet* viewImage;
};

class UserInterface final
{
public:

	VULKAN_NON_COPIABLE(UserInterface)

	UserInterface(
		Vulkan::CommandPool& commandPool, 
		const Vulkan::SwapChain& swapChain, 
		const Vulkan::DepthBuffer& depthBuffer,
		const Vulkan::RenderPass& renderPass);
	~UserInterface();

	VkExtent2D DrawUI(const Statistics& stats, UserSettings& settings);
	void Render(VkCommandBuffer commandBuffer);
	bool WantsToCaptureKeyboard() const;
	bool WantsToCaptureMouse() const;

private:

	void DrawSettings();
	void DrawOverlay(const Statistics& stats, UserSettings& settings);

	std::unique_ptr<Vulkan::DescriptorPool> descriptorPool_;
	VkExtent2D viewportExtent;
	//std::unique_ptr<Vulkan::RenderPass> renderPass_;
};
