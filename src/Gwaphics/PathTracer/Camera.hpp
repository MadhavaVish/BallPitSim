#pragma once
#include <glm/glm.hpp>
#include "../Vulkan/Device.hpp"
#include <memory>

namespace Vulkan
{
	class Buffer;
	class Device;
	class DeviceMemory;
}

namespace Vulkan
{

	struct RayGenUBO
	{
		glm::vec4 position;
		glm::vec4 coord_x;
		glm::vec4 coord_y;
		glm::vec4 coord_z;
		glm::vec4 horizontal;
		glm::vec4 vertical;
		float invWidth;
		float invHeight;
		float focusDist;
		float aperture;
		uint32_t frameIndex;
		//uint32_t accumulate;
		//uint32_t reset;
	};
	class Camera
	{
	public:
		struct Settings
		{
			float f_stop{ 32.f };
			float focus_dist{ 1.f };
			float focal_length{ 28.f };
			float sensor_width{ 36.f };
			bool accumulate = true;
		};

		Camera(uint32_t imgWidth, uint32_t imgHeight, const Device& device) : Camera(32.f, 1.f, 50.f, 36.f, { 0.f,0.f, 23.f }, {0.0f, 0.f, -1.f}, imgWidth, imgHeight, device) {};

		Camera(float fStop, float focalDist, float focalLength, float sensorWidth, glm::vec3 pos, glm::vec3 forward, uint32_t imgWidth, uint32_t imgHeight, const Device& device);
		~Camera();

		void OnResize(uint32_t width, uint32_t height);
		void moveCamera();
		void rotateCamera();
		void getSettings();
		void updateCameraUBO();
		const VkDescriptorBufferInfo& getCameraUBOInfo() const { return uniformBufferInfo; }
		const Vulkan::Buffer& Buffer() const { return *buffer_; }

	private:
		void RecalculateView();

	public:
		Settings settings;

	private:
		glm::vec3 position{ 0.0f, 0.0f, 0.0f };
		glm::vec3 forwardDir{ 0.0f, 0.0f, 0.0f };
		glm::vec3 horizontal{ 0.0f, 0.0f, 0.0f }, vertical{ 0.0f, 0.0f, 0.0f };

		glm::vec3 _x{ 0.0f, 0.0f, 0.0f }, _y{ 0.0f, 0.0f, 0.0f }, _z{ 0.0f, 0.0f, 0.0f };

		float aspect{ 0.f };
		float aperture{ 1.f };
		glm::vec2 m_LastMousePosition{ 0.0f, 0.0f };

		uint32_t m_ViewportWidth = 0, m_ViewportHeight = 0;
		float invWidth, invHeight;
		RayGenUBO cameraUBO;
		bool needsUpdate = true;

		VkDescriptorBufferInfo uniformBufferInfo = {};
		std::unique_ptr<Vulkan::Buffer> buffer_;
		std::unique_ptr<Vulkan::DeviceMemory> memory_;
	};
}