#pragma once

#include "Vulkan.hpp"
#include <vector>
namespace Vulkan
{
	class DescriptorSetLayout;
	class Device;

	class PipelineLayout final
	{
	public:

		VULKAN_NON_COPIABLE(PipelineLayout)

		PipelineLayout(const Device& device, const std::vector<VkDescriptorSetLayout> descriptorSetLayouts);
		~PipelineLayout();

	private:

		const Device& device_;

		VULKAN_HANDLE(VkPipelineLayout, pipelineLayout_)
	};

}
