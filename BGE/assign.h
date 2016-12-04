#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class assign :
		public Game
	{
	private:

	public:
		assign(void);
		~assign(void);
		bool Initialise();
		void Update();
		void Cleanup();
	    void CreateDog(glm::vec3 position);
		void CreateMan(glm::vec3 position);

	};
}