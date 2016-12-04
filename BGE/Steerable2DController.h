#pragma once
#include "GameComponent.h"
#include "Model.h"

using namespace std;
namespace BGE
{
	class Steerable2DController :
		public GameComponent
	{
	private:
		void Steerable2DController::CalculateInertiaTensor();
		shared_ptr<Model> model;
	public:
		Steerable2DController();
		Steerable2DController(shared_ptr<Model> model);
		~Steerable2DController(void);

		bool Initialise();
		void Update(float timeDelta);
		void Draw();

		SDL_Keycode keyUp = SDL_SCANCODE_UP;
		SDL_Keycode keyLeft = SDL_SCANCODE_LEFT;
		SDL_Keycode keyRight = SDL_SCANCODE_RIGHT;
		SDL_Keycode keyDown = SDL_SCANCODE_DOWN;

		float mass;
		glm::vec3 velocity, force, acceleration;
		glm::vec2 angularVelocity, angularAcceleration, torque;
		glm::mat2 inertialTensor;

		void AddForce(glm::vec3);
	};
}