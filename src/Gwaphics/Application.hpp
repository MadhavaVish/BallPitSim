#pragma once

#include "Vulkan/FrameBuffer.hpp"
#include "Vulkan/WindowConfig.hpp"
#include "Pipelines/UniformBuffer.hpp"

#include "UserInterface.hpp"
#include <vector>
#include <memory>
#include <chrono>

namespace Assets
{
	class Scene;
	class UniformBufferObject;
	class UniformBuffer;
}

namespace Vulkan 
{
	class Application
	{
	public:

		VULKAN_NON_COPIABLE(Application)

		Application(const WindowConfig& windowConfig, VkPresentModeKHR presentMode, bool enableValidationLayers);
		virtual ~Application();

		const std::vector<VkExtensionProperties>& Extensions() const;
		const std::vector<VkLayerProperties>& Layers() const;
		const std::vector<VkPhysicalDevice>& PhysicalDevices() const;

		const class SwapChain& SwapChain() const { return *swapChain_; }
		class Window& Window() { return *window_; }
		const class Window& Window() const { return *window_; }

		bool HasSwapChain() const { return swapChain_.operator bool(); }

		void SetPhysicalDevice(VkPhysicalDevice physicalDevice);
		void Run();

	protected:

		const class Device& Device() const { return *device_; }
		class CommandPool& CommandPool() { return *commandPool_; }
		const class DepthBuffer& DepthBuffer() const { return *depthBuffer_; }
		const class RenderPass& RenderPass() const { return *renderPass_; }
		const std::vector<Vulkan::UniformBuffer>& UniformBuffers() const { return quadUniformBuffers_; }
		const class FrameBuffer& SwapChainFrameBuffer(const size_t i) const { return swapChainFramebuffers_[i]; }
		
		void SetPhysicalDevice(
			VkPhysicalDevice physicalDevice, 
			std::vector<const char*>& requiredExtensions, 
			VkPhysicalDeviceFeatures& deviceFeatures,
			void* nextDeviceFeatures);
		
		void OnDeviceSet();
		void CreateSwapChain();
		void DeleteSwapChain();
		void CreateViewport();
		void DeleteViewPort();
		void DrawFrame();
		void ComputePathTrace(VkCommandBuffer commandBuffer);
		void Render(VkCommandBuffer commandBuffer, uint32_t imageIndex);

		void OnKey(int key, int scancode, int action, int mods) { }
		void OnCursorPosition(double xpos, double ypos) { }
		void OnMouseButton(int button, int action, int mods) { }
		void OnScroll(double xoffset, double yoffset) { }

	private:
		void buildComputeCommmandBuffer();

		void createComputeTargetImage();
		void deleteComputeTargetImage();
		void recordComputeCommands();
		void UpdateUniformBuffer(uint32_t imageIndex);
		void RecreateSwapChain();

		const VkPresentModeKHR presentMode_;
		
		std::unique_ptr<class Window> window_;
		std::unique_ptr<class Instance> instance_;
		std::unique_ptr<class DebugUtilsMessenger> debugUtilsMessenger_;
		std::unique_ptr<class Surface> surface_;
		std::unique_ptr<class Device> device_;
		std::unique_ptr<class SwapChain> swapChain_;
		//std::vector<Assets::UniformBuffer> uniformBuffers_;
		std::unique_ptr<class DepthBuffer> depthBuffer_;
		std::unique_ptr<class RenderPass> renderPass_;
		std::unique_ptr<class UserInterface> userInterface_;
		//std::unique_ptr<class GraphicsPipeline> graphicsPipeline_;
		std::vector<class FrameBuffer> swapChainFramebuffers_;
		std::unique_ptr<class CommandPool> commandPool_;
		std::unique_ptr<class CommandBuffers> commandBuffers_;

		bool viewportInit = false;
		bool dsetsInit = false;
		VkExtent2D viewportExtent_ = { 1, 1 };
		VkExtent2D newViewportExtent_ = { 1, 1 };
		std::unique_ptr<class DepthBuffer> viewportDepthBuffer_;
		std::unique_ptr<class RenderPass> viewportRenderPass_;
		std::vector<class Image> viewportImages_;
		std::vector<class DeviceMemory> viewportImageMemory_;
		std::vector<std::unique_ptr<class ImageView>> viewportImageViews_;
		std::vector<std::unique_ptr<class Sampler>> viewportSamplers_;
		std::vector<class FrameBuffer> viewportFramebuffers_;
		std::vector<VkDescriptorSet> viewportDsets_;

		std::vector<class Semaphore> imageAvailableSemaphores_;
		std::vector<class Semaphore> renderFinishedSemaphores_;
		std::vector<class Fence> inFlightFences_;

		std::vector<Vulkan::UniformBuffer> quadUniformBuffers_;
		std::unique_ptr<class SimpleQuadPipeline> quadPipeline_;

		int imgWidth = 1280, imgHeight = 720;
		int prevImgWidth = 1280, prevImgHeight = 720;
		float vignette = 0.01f;
		std::unique_ptr<class Image> computeImage_;
		std::unique_ptr<class DeviceMemory> computeImageMemory_;
		std::unique_ptr<class ImageView> computeImageView_;
		std::unique_ptr<class Sampler> computeSampler_;
		VkDescriptorImageInfo computeImageDescriptorInfo_;

		std::unique_ptr<class ComputeTracer> computeTracer_;
		std::unique_ptr<class CommandPool> computeCommandPool_;
		std::unique_ptr<class CommandBuffers> computeCommandBuffers_;
		std::unique_ptr<class Fence> computeFence_;

		Statistics frameStats;
		UserSettings settings;
		std::chrono::steady_clock::time_point prevTime;
		size_t currentFrame_{};
	};

}
