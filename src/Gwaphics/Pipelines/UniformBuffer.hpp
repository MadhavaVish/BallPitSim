#pragma once

#include <memory>
#include <glm/glm.hpp>

namespace Vulkan
{
	class Buffer;
	class Device;
	class DeviceMemory;
}

namespace Vulkan
{
	class UniformBufferObject
	{
	public:
		glm::mat4 ModelView;
		glm::mat4 Projection;
		float Particlescale;
	};

	class UniformBuffer
	{
	public:

		UniformBuffer(const UniformBuffer&) = delete;
		UniformBuffer& operator = (const UniformBuffer&) = delete;
		UniformBuffer& operator = (UniformBuffer&&) = delete;

		explicit UniformBuffer(const Vulkan::Device& device);
		UniformBuffer(UniformBuffer&& other) noexcept;
		~UniformBuffer();

		const Vulkan::Buffer& Buffer() const { return *buffer_; }

		void SetValue(const UniformBufferObject& ubo);

	private:

		std::unique_ptr<Vulkan::Buffer> buffer_;
		std::unique_ptr<Vulkan::DeviceMemory> memory_;
	};

}