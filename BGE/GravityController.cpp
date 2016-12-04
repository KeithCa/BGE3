#include "GravityController.h"

using namespace BGE;

GravityController::GravityController(){

	gravity = glm::vec3(0, -0.5f, 0);
}

void GravityController::Update(float timeDelta){
	// Apply force to y.
	if ((parent->transform->position.y <= 0) && parent->transform->velocity.y < 0){

		parent->transform->velocity.y = -parent->transform->velocity.y * .8f;
		parent->transform->position.y = 0 + parent->transform->scale.y;
	}
	else{
		parent->transform->position += gravity * timeDelta * .7f;
		parent->transform->velocity += gravity * timeDelta* .7f;
	}

	parent->transform->position.y += parent->transform->velocity.y;
}