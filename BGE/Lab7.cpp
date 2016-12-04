#include "Lab7.h"
#include "Content.h"
#include "VectorDrawer.h"
#include "GravityController.h"
#include "Utils.h"


using namespace BGE;

Lab7::Lab7(){

}

bool Lab7::Initialise(){
	std::shared_ptr<GameComponent> ground = make_shared<Ground>();
	Attach(ground);

	ship1 = make_shared<GameComponent>(true);
	ship1->Attach(Content::LoadModel("cobramk3", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0, 1, 0))));
	ship1->transform->position = glm::vec3(-10, 2, -10);
	ship1->Attach(make_shared<VectorDrawer>());
	ship1->Attach(make_shared<GravityController>());
	Attach(ship1);

	Game::Initialise();

	camera->transform->position = glm::vec3(0, 4, 20);
	return true;
}

void Lab7::Update(float timeDelta){

	Game::Update(timeDelta);
}
