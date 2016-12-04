#include "Content.h"
#include "VectorDrawer.h"
#include "GravityController.h"
#include "Box.h"
#include "Sphere.h"
#include "Capsule.h"
#include "Cylinder.h"
#include "Steerable2DController.h"
#include "Steerable3DController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>
#include "Utils.h"

#include "assign.h"
using namespace BGE;



assign::assign(void)
{
}

assign::~assign(void)
{
}



bool assign::Initialise()
{

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	dynamicsWorld->setGravity(btVector3(0, -9.0f, 0));

	
	CreateMan(glm::vec3(20, 9, 0));
	CreateDog(glm::vec3(0, 0, 0));


	return Game::Initialise();
}

void BGE::assign::Update()
{


}

void BGE::assign::Cleanup()
{
	Game::Cleanup();
}

void BGE::assign::CreateMan(glm::vec3 position){
	float bodyX = 4.0f;
	float bodyY = 7.0f;
	glm::quat q = glm::angleAxis(-90.0f, glm::vec3(0, 1, 0));
	glm::quat q2 = glm::angleAxis(-90.0f, glm::vec3(0, 0, 1));
	std::shared_ptr<GameComponent> base_ = make_shared<Box>(bodyX, bodyY, 2.0f);
	std::shared_ptr<PhysicsController> base = physicsFactory->CreateBox(bodyX, bodyY, 2.0f, position, base_->transform->orientation, false, true);
	std::shared_ptr<PhysicsController> rleg = physicsFactory->CreateCapsule(.5, 1.5f, glm::vec3(position.x + 3, position.y - 4.5f, position.z), q);
	std::shared_ptr<PhysicsController> lleg = physicsFactory->CreateCapsule(.5, 1.5f, glm::vec3(position.x - 3, position.y - 4.5f, position.z), q);
	std::shared_ptr<PhysicsController> rhand = physicsFactory->CreateCapsule(.5, 1.5f, glm::vec3(position.x + 5, position.y + 2.5f, position.z), q2);
	std::shared_ptr<PhysicsController> lhand = physicsFactory->CreateCapsule(.5, 1.5f, glm::vec3(position.x - 5, position.y + 2.5f, position.z), q2);


	std::shared_ptr<PhysicsController> head = physicsFactory->CreateSphere(2.0f, glm::vec3(position.x, position.y + 5.5f, position.z), glm::quat());
	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -2, 0));
	t2.setOrigin(btVector3(0, 4, 0));

	btFixedConstraint * headfixed = new btFixedConstraint(*head->rigidBody, *base->rigidBody, t1, t2);
	btHingeConstraint *rleghinge = new btHingeConstraint(*rleg->rigidBody, *base->rigidBody, btVector3(0, 1.5f, 0), btVector3(3, -3, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *lleghinge = new btHingeConstraint(*lleg->rigidBody, *base->rigidBody, btVector3(0, 1.5f, 0), btVector3(-3, -3, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *rhandhinge = new btHingeConstraint(*rhand->rigidBody, *base->rigidBody, btVector3(0, -1.5f, 0), btVector3(3, 3, 0), btVector3(1, 1, 1), btVector3(1, 1, 1), false);
	btHingeConstraint *lhandhinge = new btHingeConstraint(*lhand->rigidBody, *base->rigidBody, btVector3(0, 1.5f, 0), btVector3(-3, 3, 0), btVector3(1, 1, 1), btVector3(1, 1, 1), false);


	rleghinge->enableAngularMotor(true, 20.0f, 20.0f);
	lleghinge->enableAngularMotor(true, 20.0f, 20.0f);

	dynamicsWorld->addConstraint(headfixed);
	dynamicsWorld->addConstraint(lleghinge);
	dynamicsWorld->addConstraint(rleghinge);
	dynamicsWorld->addConstraint(lhandhinge);
	dynamicsWorld->addConstraint(rhandhinge);
}

void BGE::assign::CreateDog(glm::vec3 position)
{
	float x = -10;
	float z = 20;

	glm::quat q = glm::angleAxis(-90.0f, glm::vec3(1, 0, 0));
	shared_ptr<PhysicsController> body = physicsFactory->CreateCapsule(2, 2.5, glm::vec3(x, 7, z), q);
	shared_ptr<PhysicsController> head = physicsFactory->CreateSphere(2.0f, glm::vec3(x, 7, z), glm::quat());
	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -3, 0));
	t2.setRotation(GLToBtQuat(glm::angleAxis(-10.0f, glm::vec3(2.5, 1, 0)))); 
	t2.setOrigin(btVector3(0, 3, 0));
	btFixedConstraint * headfixed = new btFixedConstraint(*head->rigidBody, *body->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(headfixed);

	shared_ptr<PhysicsController> frontleftshoulderjoint = physicsFactory->CreateCylinder(.5, .5, glm::vec3(x, 13, z), glm::quat());
	shared_ptr<PhysicsController> frontleftleg = physicsFactory->CreateCapsule(.75f, 1, glm::vec3(x + 2.5, 7, z), glm::quat());
	shared_ptr<PhysicsController>  frontrightshoulderjoint = physicsFactory->CreateCylinder(.5, .5, glm::vec3(x, 13, z), glm::quat());
	shared_ptr<PhysicsController> frontrightleg = physicsFactory->CreateCapsule(.75f, 1, glm::vec3(x + 2.5, 7, z), glm::quat());
	shared_ptr<PhysicsController> backleftshoulderjoint = physicsFactory->CreateCylinder(.5, .5, glm::vec3(x, 13, z), glm::quat());
	shared_ptr<PhysicsController> backleftleg = physicsFactory->CreateCapsule(.75, 1, glm::vec3(x + 2.5, 7, z), glm::quat());
	shared_ptr<PhysicsController> backrightshoulderjoint = physicsFactory->CreateCylinder(.5, .5, glm::vec3(x, 13, z), glm::quat());
	shared_ptr<PhysicsController> backrightleg = physicsFactory->CreateCapsule(.75, 1, glm::vec3(x + 2.5, 7, z), glm::quat());

	btHingeConstraint *frontleftshoulder = new btHingeConstraint(*frontleftshoulderjoint->rigidBody, *body->rigidBody, btVector3(0, .5, 0), btVector3(2, 3.5, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *frontleftshoulderleg = new btHingeConstraint(*frontleftleg->rigidBody, *frontleftshoulderjoint->rigidBody, btVector3(0, 3.5, 0), btVector3(1, 1, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *frontrightshoulder = new btHingeConstraint(*frontrightshoulderjoint->rigidBody, *body->rigidBody, btVector3(0, -.5, 0), btVector3(-2, 3.5, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *frontrightshoulderleg = new btHingeConstraint(*frontrightleg->rigidBody, *frontrightshoulderjoint->rigidBody, btVector3(0, 3.5, 0), btVector3(1, -1, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *backleftshoulder = new btHingeConstraint(*backleftshoulderjoint->rigidBody, *body->rigidBody, btVector3(0, -.5, 0), btVector3(-2, -3.5, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *backleftshoulderleg = new btHingeConstraint(*backleftleg->rigidBody, *backleftshoulderjoint->rigidBody, btVector3(0, 3.5, 0), btVector3(1, -1, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *backrightshoulder = new btHingeConstraint(*backrightshoulderjoint->rigidBody, *body->rigidBody, btVector3(0, .5, 0), btVector3(2, -3.5, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);
	btHingeConstraint *backrightshoulderleg = new btHingeConstraint(*backrightleg->rigidBody, *backrightshoulderjoint->rigidBody, btVector3(0, 3.5, 0), btVector3(1, 1, 0), btVector3(0, 1, 0), btVector3(1, 0, 0), false);

	float MaxMotor = -50.0f;
	bool IsEnabled = true;
	float Low = -1.5f;
	float High = 1.5f;

	frontleftshoulder->enableMotor(IsEnabled);
	frontleftshoulder->setMaxMotorImpulse(MaxMotor);
	frontleftshoulder->setLimit(Low, High);
	frontrightshoulder->enableMotor(IsEnabled);
	frontrightshoulder->setMaxMotorImpulse(MaxMotor);
	frontrightshoulder->setLimit(Low, High);
	backleftshoulder->enableMotor(IsEnabled);
	backleftshoulder->setMaxMotorImpulse(MaxMotor);
	backleftshoulder->setLimit(Low, High);
	backrightshoulder->enableMotor(IsEnabled);
	backrightshoulder->setMaxMotorImpulse(MaxMotor);
	backrightshoulder->setLimit(Low, High);	
	
	dynamicsWorld->addConstraint(frontleftshoulder);
	dynamicsWorld->addConstraint(frontleftshoulderleg);
	dynamicsWorld->addConstraint(frontrightshoulder);
	dynamicsWorld->addConstraint(frontrightshoulderleg);
	dynamicsWorld->addConstraint(backleftshoulder);
	dynamicsWorld->addConstraint(backleftshoulderleg);
	dynamicsWorld->addConstraint(backrightshoulder);
	dynamicsWorld->addConstraint(backrightshoulderleg);


	shared_ptr<PhysicsController> tail = physicsFactory->CreateCapsule(.5, .5, glm::vec3(x + 2.5, 4, z - 2.5), glm::quat());
	btHingeConstraint * hingetail = new btHingeConstraint(*tail->rigidBody, *body->rigidBody, btVector3(0, 2, 0), btVector3(0, -5, 0), btVector3(1, 1, 1), btVector3(0, 1, 0), false);
	dynamicsWorld->addConstraint(hingetail);
	//hingetail->enableAngularMotor(true, 20.0f, 20.0f);

}