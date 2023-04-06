#include "Camera.hpp"
#include "../Vulkan/Buffer.hpp"
#include <iostream>
namespace Vulkan
{
	Camera::Camera(float fStop, float focalDist, float focalLength, float sensorWidth, glm::vec3 pos, glm::vec3 forward, uint32_t imgWidth, uint32_t imgHeight, const Device& device)
		:settings{ fStop, focalDist, focalLength, sensorWidth },
		forwardDir(forward),
		position(pos)
	{
		const auto bufferSize = sizeof(RayGenUBO);

		buffer_.reset(new Vulkan::Buffer(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT));
		memory_.reset(new Vulkan::DeviceMemory(buffer_->AllocateMemory(VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)));
		
		uniformBufferInfo.buffer = buffer_->Handle();
		uniformBufferInfo.range = VK_WHOLE_SIZE;
		OnResize(imgWidth, imgHeight);
		//std::cout << "camera made it" << std::endl;
	}
	Camera::~Camera()
	{
		buffer_.reset();
		memory_.reset();
	}

	void Camera::OnResize(uint32_t width, uint32_t height)
	{
		if (width == m_ViewportWidth && height == m_ViewportHeight)
		{
			return;
		}
		m_ViewportWidth = width;
		m_ViewportHeight = height;
		invHeight = 1.f / (float)m_ViewportHeight;
		invWidth = 1.f / (float)m_ViewportWidth;
		aspect = (float)m_ViewportHeight / (float)m_ViewportWidth;
		RecalculateView();
	}

	void Camera::updateCameraUBO()
	{
		if (needsUpdate)
		{
			cameraUBO.position = glm::vec4(position,0.f);
			cameraUBO.coord_x = glm::vec4(_x, 0.f);
			cameraUBO.coord_y = glm::vec4(_y, 0.f);
			cameraUBO.coord_z = glm::vec4(_z, 0.f);

			cameraUBO.horizontal = glm::vec4(horizontal, 0.f);
			cameraUBO.vertical = glm::vec4(vertical, 0.f);

			cameraUBO.invWidth = invWidth;
			cameraUBO.invHeight = invHeight;
			
			cameraUBO.focusDist = settings.focus_dist;
			cameraUBO.aperture = aperture;
			
			cameraUBO.frameIndex = 1;
			//cameraUBO.accumulate = settings.accumulate;
			//cameraUBO.reset = settings.reset;

			const auto data = memory_->Map(0, sizeof(RayGenUBO));
			std::memcpy(data, &cameraUBO, sizeof(cameraUBO));
			memory_->Unmap();
			needsUpdate = false;
			return;
		}
		else
		{
			cameraUBO.frameIndex += 1;
			const auto data = memory_->Map(0, sizeof(RayGenUBO));
			std::memcpy(data, &cameraUBO, sizeof(cameraUBO));
			memory_->Unmap();
		}
	}

	void Camera::RecalculateView()
	{
		float w = settings.sensor_width * 0.001f / (2 * settings.focal_length * 0.001f);
		aperture = 0.5f * settings.focal_length * 0.001f / settings.f_stop;
		float width = 2 * w;
		float height = aspect * width;

		_z = normalize(forwardDir);
		_x = glm::normalize(glm::cross(_z, { 0.f, 1.f, 0.f }));
		_y = glm::cross(_z, _x);

		horizontal = settings.focus_dist * width * _x / 2.f;
		vertical = settings.focus_dist * height * _y / 2.f;
		needsUpdate = true;
	}
}