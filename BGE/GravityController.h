#pragma once
#include "GameComponent.h"
#include "Game.h"

namespace BGE
{
	class GravityController :public GameComponent{

		std::shared_ptr<GameComponent> ground = make_shared<Ground>();
	public: GravityController();
			glm::vec3 gravity;
			void Update(float timeDelta);
	};

}