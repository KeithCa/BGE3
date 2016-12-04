#pragma once
#include "GameComponent.h"
#include "Model.h"

using namespace std;
namespace BGE
{
	class Steerable3DController :
		public GameComponent
	{
	private:
		void Steerable3DController::CalculateInertiaTensor();
		shared_ptr<Model> model;
	public:
		Steerable3DController(shared_ptr<Model> model);
		~Steerable3DController(void);

		bool Initialise();
		void Update(float timeDelta);
		void Draw();

		SDL_Keycode keyUp = SDL_SCANCODE_B;
		SDL_Keycode keyLeft = SDL_SCANCODE_J;
		SDL_Keycode keyRight = SDL_SCANCODE_L;
		SDL_Keycode keyDown = SDL_SCANCODE_K;
		SDL_Keycode keyForward = SDL_SCANCODE_SPACE;

		float mass;
		glm::vec3 velocity, force, acceleration;
        glm::vec3 angularVelocity, angularAcceleration, torque;
        glm::mat3 inertialTensor;

		void AddForce(glm::vec3);
		void AddTorque(glm::vec3);
		void AddForceAtPoint(glm::vec3, glm::vec3);
	};
}
