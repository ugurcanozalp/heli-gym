#include "camera.h"
#include <iostream>

Camera::Camera(glm::vec3 position, 
				glm::vec3 up, 
				float yaw, 
				float pitch)
{
	Front = glm::vec3(0.0f, 0.0f, -1.0f);
	this->Position = position;
	this->WorldUp = up;
	this->Yaw = yaw;
	this->Pitch = pitch;
	updateCameraVectors();
}

Camera::Camera(
		float posX, 
		float posY, 
		float posZ, 
		float upX, 
		float upY, 
		float upZ, 
		float yaw, 
		float pitch) 
{
	Front = glm::vec3(0.0f, 0.0f, -1.0f);
	this->Position = glm::vec3(posX, posY, posZ);
	this->WorldUp = glm::vec3(upX, upY, upZ);
	this->Yaw = yaw;
	this->Pitch = pitch;
	updateCameraVectors();
}

glm::mat4 Camera::GetViewMatrix()
{
	return glm::lookAt(Position, Position + Front, Up);
}

void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime)
{
	// Set boost of camera. 
	boost = 1.0f;
	if (direction == BOOST)
		boost *= 50.5f;

	// Set velocity of the camera.
	float velocity = MovementSpeed * deltaTime * boost;
	
	// Set new position of camera.
	if (direction == FORWARD)
		Position += Front * velocity;
	if (direction == BACKWARD)
		Position -= Front * velocity;
	if (direction == LEFT)
		Position -= Right * velocity;
	if (direction == RIGHT)
		Position += Right * velocity;
	if (direction == UP)
		Position += Up * velocity;
	if (direction == DOWN)
		Position -= Up * velocity;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch)
{
	// Set new offset of mouse w.r.t mouse sensitivity.
	xoffset *= MouseSensitivity;
	yoffset *= MouseSensitivity;

	// Change camera angles.
	this->Yaw += xoffset;
	this->Pitch += yoffset;

	// Make sure that when pitch is out of bounds, screen doesn't get flipped.
	if (constrainPitch)
	{
		if (Pitch > 89.0f)
			Pitch = 89.0f;
		if (Pitch < -89.0f)
			Pitch = -89.0f;
	}

	// Update Front, Right and Up Vectors using the updated Euler angles
	updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset)
{
	// Set zoom value of camera.
	Zoom -= (float)yoffset;
	if (Zoom < 1.0f)
		Zoom = 1.0f;
	if (Zoom > 45.0f)
		Zoom = 45.0f;
}

void Camera::updateCameraVectors()
{
	// Calculate the new Front vector
	glm::vec3 front;
	front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	front.y = sin(glm::radians(Pitch));
	front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
	Front = glm::normalize(front);

	// also re-calculate the Right and Up vector
	Right = glm::normalize(glm::cross(Front, WorldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
	Up = glm::normalize(glm::cross(Right, Front));
}