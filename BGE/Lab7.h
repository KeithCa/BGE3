#pragma once
#include "Game.h"
#include "GameComponent.h"

using namespace std;

namespace BGE{

	class Lab7 : public Game{

		shared_ptr<GameComponent> ship1;

	public: Lab7(void);
			bool Initialise();
			void Update(float timeDelta);
	};
}