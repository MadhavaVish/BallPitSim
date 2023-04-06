#include "ComputeTracer.hpp"


#include "../Vulkan/ShaderModule.hpp"
#include "../Vulkan/DescriptorSets.hpp"
#include "../Vulkan/Image.hpp"
#include "../Vulkan/ImageView.hpp"
#include "../Vulkan/Sampler.hpp"
#include "../Vulkan/BufferUtil.hpp"

#include <memory>
#include "vulkan/vulkan.hpp"

namespace Vulkan
{
	ComputeTracer::ComputeTracer(const Vulkan::Device& device, Vulkan::CommandPool& commandPool, VkDescriptorImageInfo& imageInfo, uint32_t imgWidth, uint32_t imgHeight)
	:device_(device), 
	commandPool_(commandPool),
	camera_(imgWidth, imgHeight, device)
	{
		//const auto& device = swapChain.Device();

		createAccumulatorImage(imgWidth, imgHeight);
		BufferUtil::CreateDeviceBuffer(commandPool, "Vertices", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.vertices, vertexBuffer_, vertexBufferMemory_);
		VkDescriptorBufferInfo vertexBufferInfo = {};
		vertexBufferInfo.buffer = vertexBuffer_->Handle();
		vertexBufferInfo.range = VK_WHOLE_SIZE;

		BufferUtil::CreateDeviceBuffer(commandPool, "Normals", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.normals, normalBuffer_, normalBufferMemory_);
		VkDescriptorBufferInfo normalBufferInfo = {};
		normalBufferInfo.buffer = normalBuffer_->Handle();
		normalBufferInfo.range = VK_WHOLE_SIZE;

		BufferUtil::CreateDeviceBuffer(commandPool, "Indices", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.indices, indexBuffer_, indexBufferMemory_);
		VkDescriptorBufferInfo indexBufferInfo = {};
		indexBufferInfo.buffer = indexBuffer_->Handle();
		indexBufferInfo.range = VK_WHOLE_SIZE;

		BufferUtil::CreateDeviceBuffer(commandPool, "Triangles", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.triangles, triangleBuffer_, triangleBufferMemory_);
		VkDescriptorBufferInfo triangleBufferInfo = {};
		triangleBufferInfo.buffer = triangleBuffer_->Handle();
		triangleBufferInfo.range = VK_WHOLE_SIZE;

		BufferUtil::CreateDeviceBuffer(commandPool, "BVHNode", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.bvhNode, bvhNodeBuffer_, bvhNodeBufferMemory_);
		VkDescriptorBufferInfo bvhNodeBufferInfo = {};
		bvhNodeBufferInfo.buffer = bvhNodeBuffer_->Handle();
		bvhNodeBufferInfo.range = VK_WHOLE_SIZE;

		BufferUtil::CreateDeviceBuffer(commandPool, "Materials", VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, scene.materials, materialBuffer_, materialBufferMemory_);
		VkDescriptorBufferInfo materialBufferInfo = {};
		materialBufferInfo.buffer = materialBuffer_->Handle();
		materialBufferInfo.range = VK_WHOLE_SIZE;

		const std::vector<DescriptorBinding> descriptorBindings =
		{
			{0, 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT},
			{1, 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT},
			{2, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{3, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{4, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{5, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{6, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{7, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
			{8, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT},
		};

		descriptorSetManager_.reset(new DescriptorSetManager(device, descriptorBindings,1));
		auto& descriptorSets = descriptorSetManager_->DescriptorSets();
		std::vector<VkWriteDescriptorSet> descriptorWrites;
		descriptorWrites.push_back(descriptorSets.Bind(0, 0, imageInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 1, accumulatorImageDescriptorInfo_));
		descriptorWrites.push_back(descriptorSets.Bind(0, 2, camera_.getCameraUBOInfo()));
		descriptorWrites.push_back(descriptorSets.Bind(0, 3, vertexBufferInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 4, indexBufferInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 5, triangleBufferInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 6, bvhNodeBufferInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 7, normalBufferInfo));
		descriptorWrites.push_back(descriptorSets.Bind(0, 8, materialBufferInfo));

		descriptorSets.UpdateDescriptors(0, descriptorWrites);

		std::vector<VkDescriptorSetLayout> pipelineLayouts;
		pipelineLayouts.push_back(descriptorSetManager_->DescriptorSetLayout().Handle());

		pipelineLayout_.reset(new class PipelineLayout(device, pipelineLayouts));

		const ShaderModule computeShader(device, "assets/shaders/tracer.comp.spv");

		VkPipelineShaderStageCreateInfo shaderStage = computeShader.CreateShaderStage(VK_SHADER_STAGE_COMPUTE_BIT);

		VkComputePipelineCreateInfo computeCreateInfo = {};
		computeCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
		computeCreateInfo.layout = pipelineLayout_->Handle();
		computeCreateInfo.stage = shaderStage;

		vkCreateComputePipelines(device.Handle(), nullptr, 1, &computeCreateInfo, nullptr, &pipeline_);
	}

	ComputeTracer::~ComputeTracer()
	{
		if (pipeline_ != nullptr)
		{
			vkDestroyPipeline(device_.Handle(), pipeline_, nullptr);
			pipeline_ = nullptr;
		}

		pipelineLayout_.reset();
		descriptorSetManager_.reset();
	}

	void ComputeTracer::resizeComputeTarget(uint32_t imgWidth, uint32_t imgHeight, VkDescriptorImageInfo& imageDescriptor)
	{
		camera_.OnResize(imgWidth, imgHeight);

		deleteAccumulatorImage();
		createAccumulatorImage(imgWidth, imgHeight);

		auto& descriptorSets = descriptorSetManager_->DescriptorSets();
		std::vector<VkWriteDescriptorSet> descriptorWrites;
		descriptorWrites.push_back(descriptorSets.Bind(0, 0, imageDescriptor));
		descriptorWrites.push_back(descriptorSets.Bind(0, 1, accumulatorImageDescriptorInfo_));

		descriptorSets.UpdateDescriptors(0, descriptorWrites);
	}

	VkDescriptorSet ComputeTracer::ComputeTextureDescriptorSet() const
	{
		return descriptorSetManager_->DescriptorSets().Handle(0);
	}
	void ComputeTracer::createAccumulatorImage(uint32_t imgWidth, uint32_t imgHeight)
	{
		accumulatorImage_ = std::make_unique<Image>(device_, VkExtent2D(imgWidth, imgHeight), VK_FORMAT_R32G32B32A32_SFLOAT, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_STORAGE_BIT);
		accumulatorImageMemory_ = std::make_unique<DeviceMemory>(accumulatorImage_->AllocateMemory(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
		accumulatorImage_->TransitionImageLayout(commandPool_, VK_IMAGE_LAYOUT_GENERAL);
		//accumulatorImageSampler_ = std::make_unique<Sampler>(device_, SamplerConfig{});
		accumulatorImageView_ = std::make_unique<ImageView>(device_, accumulatorImage_->Handle(), accumulatorImage_->Format(), VK_IMAGE_ASPECT_COLOR_BIT);
		accumulatorImageDescriptorInfo_ = {};
		accumulatorImageDescriptorInfo_.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
		accumulatorImageDescriptorInfo_.imageView = accumulatorImageView_->Handle();
	}
	void ComputeTracer::deleteAccumulatorImage()
	{
		device_.WaitIdle();
		//accumulatorImageSampler_.reset();
		accumulatorImageView_.reset();
		accumulatorImage_.reset();
		accumulatorImageMemory_.reset();
	}
}