
#include "Gwaphics/Vulkan/Enumerate.hpp"
#include "Gwaphics/Vulkan/Strings.hpp"
#include "Gwaphics/Vulkan/SwapChain.hpp"
#include "Gwaphics/Vulkan/Version.hpp"
#include "Gwaphics/Utilities/Console.hpp"
#include "Gwaphics/Application.hpp"
#include "UserSettings.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>

namespace
{
	UserSettings CreateUserSettings();
	void PrintVulkanSdkInformation();
	void PrintVulkanInstanceInformation(const Vulkan::Application& application);
	void PrintVulkanLayersInformation(const Vulkan::Application& application);
	void PrintVulkanDevices(const Vulkan::Application& application);
	void PrintVulkanSwapChainInformation(const Vulkan::Application& application);
	void SetVulkanDevice(Vulkan::Application& application);
}

int main(int argc, const char* argv[]) noexcept
{
	try
	{
		Vulkan::WindowConfig windowConfig
		{
			"Vulkan Window",
		};
		windowConfig.CursorDisabled = false;
		windowConfig.Fullscreen = false;
		windowConfig.Height = 720U;
		windowConfig.Width = 1280U;
		windowConfig.Resizable = true;
		windowConfig.Title = std::string("Big Nice");
		const bool EnableValidationLayers =
		#ifdef _DEBUG
					true;
		#else
					false;
		#endif
		Vulkan::Application application(windowConfig, VkPresentModeKHR::VK_PRESENT_MODE_MAILBOX_KHR, EnableValidationLayers);

		PrintVulkanSdkInformation();
		//PrintVulkanInstanceInformation(application);
		//PrintVulkanLayersInformation(application);
		//PrintVulkanDevices(application);

		SetVulkanDevice(application);

		PrintVulkanSwapChainInformation(application);

		application.Run();

		return EXIT_SUCCESS;
	}
	catch (const std::exception& exception)
	{

		std::cerr << "FATAL: " << exception.what() << std::endl;
	}

	return EXIT_FAILURE;
}

namespace
{

	void PrintVulkanSdkInformation()
	{
		std::cout << "Vulkan SDK Header Version: " << VK_HEADER_VERSION << std::endl;
		std::cout << std::endl;
	}

	void PrintVulkanInstanceInformation(const Vulkan::Application& application)
	{

		std::cout << "Vulkan Instance Extensions: " << std::endl;

		for (const auto& extension : application.Extensions())
		{
			std::cout << "- " << extension.extensionName << " (" << Vulkan::Version(extension.specVersion) << ")" << std::endl;
		}

		std::cout << std::endl;
	}

	void PrintVulkanLayersInformation(const Vulkan::Application& application)
	{
		std::cout << "Vulkan Instance Layers: " << std::endl;

		for (const auto& layer : application.Layers())
		{
			std::cout
				<< "- " << layer.layerName
				<< " (" << Vulkan::Version(layer.specVersion) << ")"
				<< " : " << layer.description << std::endl;
		}

		std::cout << std::endl;
	}

	void PrintVulkanDevices(const Vulkan::Application& application)
	{
		std::cout << "Vulkan Devices: " << std::endl;

		for (const auto& device : application.PhysicalDevices())
		{
			VkPhysicalDeviceDriverProperties driverProp{};
			driverProp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DRIVER_PROPERTIES;

			VkPhysicalDeviceProperties2 deviceProp{};
			deviceProp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
			deviceProp.pNext = &driverProp;

			vkGetPhysicalDeviceProperties2(device, &deviceProp);

			VkPhysicalDeviceFeatures features;
			vkGetPhysicalDeviceFeatures(device, &features);

			const auto& prop = deviceProp.properties;

			const Vulkan::Version vulkanVersion(prop.apiVersion);
			const Vulkan::Version driverVersion(prop.driverVersion, prop.vendorID);

			std::cout << "- [" << prop.deviceID << "] ";
			std::cout << Vulkan::Strings::VendorId(prop.vendorID) << " '" << prop.deviceName;
			std::cout << "' (";
			std::cout << Vulkan::Strings::DeviceType(prop.deviceType) << ": ";
			std::cout << "vulkan " << vulkanVersion << ", ";
			std::cout << "driver " << driverProp.driverName << " " << driverProp.driverInfo << " - " << driverVersion;
			std::cout << ")" << std::endl;
		}

		std::cout << std::endl;
	}

	void PrintVulkanSwapChainInformation(const Vulkan::Application& application)
	{
		const auto& swapChain = application.SwapChain();

		std::cout << "Swap Chain: " << std::endl;
		std::cout << "- image count: " << swapChain.Images().size() << std::endl;
		std::cout << "- present mode: " << swapChain.PresentMode() << std::endl;
		std::cout << std::endl;
	}

	void SetVulkanDevice(Vulkan::Application& application)
	{
		const auto& physicalDevices = application.PhysicalDevices();
		const auto result = std::find_if(physicalDevices.begin(), physicalDevices.end(), [](const VkPhysicalDevice& device)
			{
				// We want a device with geometry shader support.
				VkPhysicalDeviceFeatures deviceFeatures;
		vkGetPhysicalDeviceFeatures(device, &deviceFeatures);
		VkPhysicalDeviceProperties properties{};
		vkGetPhysicalDeviceProperties(device, &properties);
		return (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU);
		//if (!deviceFeatures.geometryShader)
		//{
		//	return false;
		//}

		// We want a device that supports the ray tracing extension.
		const auto extensions = Vulkan::GetEnumerateVector(device, static_cast<const char*>(nullptr), vkEnumerateDeviceExtensionProperties);
		/*const auto hasRayTracing = std::find_if(extensions.begin(), extensions.end(), [](const VkExtensionProperties& extension)
			{
				return strcmp(extension.extensionName, VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME) == 0;
			});

		if (hasRayTracing == extensions.end())
		{
			return false;
		}*/

		// We want a device with a graphics queue.
		const auto queueFamilies = Vulkan::GetEnumerateVector(device, vkGetPhysicalDeviceQueueFamilyProperties);
		const auto hasGraphicsQueue = std::find_if(queueFamilies.begin(), queueFamilies.end(), [](const VkQueueFamilyProperties& queueFamily)
			{
				return queueFamily.queueCount > 0 && queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT;
			});

		return hasGraphicsQueue != queueFamilies.end();
			});

		if (result == physicalDevices.end())
		{
			throw(std::runtime_error("cannot find a suitable device"));
		}

		/*VkPhysicalDeviceProperties2 deviceProp{};
		deviceProp.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
		vkGetPhysicalDeviceProperties2(*result, &deviceProp);

		std::cout << "Setting Device [" << deviceProp.properties.deviceID << "]:" << std::endl;*/

		application.SetPhysicalDevice(*result);

		std::cout << std::endl;
	}

}
