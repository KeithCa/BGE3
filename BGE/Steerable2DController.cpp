#include "Steerable2DController.h"
#include "Content.h"
#include "Model.h"
#include "Utils.h"
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtc/matrix_transform.hpp>
#include <sstream>
#include <string>

using namespace BGE;
using namespace std;

Steerable2DController::Steerable2DController(){

}

Steerable2DController::Steerable2DController(shared_ptr<Model> model) :GameComponent()
{
	mass = 5.0f;
	velocity = glm::vec3(0);
	force = glm::vec3(0);
	acceleration = glm::vec3(0);
	this->model = model;
}

Steerable2DController::~Steerable2DController(void)
{
}

bool Steerable2DController::Initialise()
{
	if (!model->initialised)
	{
		model->Initialise();
	}

	GameComponent::Initialise();
	return true;
}

void Steerable2DController::AddForce(glm::vec3 force)
{
	this->force += force;
}

void Steerable2DController::Draw()
{
	GameComponent::Draw();
}

void Steerable2DController::Update(float timeDelta)
{
	// Gets Key State
	const Uint8 * keyState = Game::Instance()->GetKeyState();

	//A set force to apply 
	float std_force = 10.0f;
	float infintesimal = 0.000001;
	if (keyState[SDL_SCANCODE_UP])
	{
		//Apply the set force to the look vector to go towards where you're looking
		force += this->transform->look * std_force;
	}
	if (keyState[SDL_SCANCODE_DOWN])
	{
		//Apply se force to look vector negatively to go backwards
		force -= this->transform->look * std_force;
	}
	if (keyState[SDL_SCANCODE_LEFT])
	{
		// Apply negative force to the right vector to rotate left
		force -= this->transform->right * std_force;

	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		//Apply force to righ vector to rotate right
		force += this->transform->right * std_force;
	}

	//Calculate acceleration, velocity and thus the new position
	glm::vec3 accel = force / mass;
	this->transform->velocity += accel * timeDelta;
	this->transform->position += this->transform->velocity * timeDelta * std_force;
	// Check if the velocity length is > infintesimal and if so create the look vector from the velocity
	if (glm::length(this->transform->velocity) > infintesimal)
	{
		this->transform->look = glm::normalize(this->transform->velocity);
	}
	//Check if length between look and basisLook is greater than the 'infintesimal', an extremely small value
	if (glm::length(this->transform->look - Transform::basisLook) > infintesimal)
	{

		this->transform->position += this->transform->velocity * timeDelta;
	}
	// Apply damping
	this->transform->velocity *= 0.99f;
	// 
	force = glm::vec3(0, 0, 0);
	GameComponent::Update(timeDelta);
}