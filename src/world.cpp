#ifndef _RPWORLD_
#define _RPWORLD_

#include<iostream>
#include"world.h"

#define GET_OBJECTS_FORCE

using namespace RPEngine;

void RPWorld::startFrame()
{
	for(RigidBodies::iterator body = bodies.begin();
		body != bodies.end();
		body++)
	{
		(*body)->clearAccumulators();
		(*body)->calculateDerivedData();
	}
}

void RPWorld::runPhysics(real duration)
{
	std::cout << duration << "in engine duration";
	std::cout << MIN_FRAME_RATE<<std::endl;
	if(duration <= 0) return;
	if(duration > MIN_FRAME_RATE)
	{
		duration -= MIN_FRAME_RATE;
		std::cout << duration <<" frame division.\n";
		runPhysics(duration);
		duration = MIN_FRAME_RATE;
	}

	registry.updateForces(duration); //apply forces
	integrate(duration); // calculate pos, vel, acc all bodies 

	//contact container ready
	collisionData.reset(maxContacts);
	
	//generate contact by contactGenerator
	unsigned int usedContacts = generateContacts();

	//detect broad collision 
	//SAT or GJK, not yet.

	//detect narrow collision
	usedContacts += _narrowCollisionDetect();
	
	if(usedContacts)
	{
		_collisionEventExcute(duration); //(contactArray, usedContacts);
		_collisionTriggerExcute();
		//resolve collision
		if(calculateIterations) resolver.setIterations(usedContacts * ITERATIONS_PER_CONTACT);
		resolver.resolveContacts(contactArray, usedContacts, duration);
	}
	
}

void RPWorld::_broadCollisionDetect()
{
	//BSP, BVH 구현
	//narrow를 위한 중간 데이터를 남겨야 한다.
}

void RPWorld::_collisionTriggerExcute()
{

}

//not using broad collision route
int RPWorld::_narrowCollisionDetect()
{
	//광역탐지를 통한 데이터만을 탐지
	//detect collision primitives
	int usedContact = 0;
	CollisionBoxes::iterator iterBox = collisionBoxes.begin();

	//box-any collisionModel
	for(; iterBox != collisionBoxes.end(); iterBox++)
	{
		CollisionPlanes::iterator iterPlane = collisionPlanes.begin();
		for(; iterPlane != collisionPlanes.end(); iterPlane++)
		{
			(*iterBox)->calculateInternals();
			usedContact += CollisionDetector::box_halfSpace(**iterBox, **iterPlane, &collisionData);
		}

		CollisionBoxes::iterator iterBox2 = iterBox;
		for(; iterBox2 != collisionBoxes.end(); iterBox2++)
		{
			if(*iterBox == *iterBox2) continue;
			(*iterBox)->calculateInternals();
			(*iterBox2)->calculateInternals();
			usedContact += CollisionDetector::box_box(**iterBox, **iterBox2, &collisionData);
		}

		CollisionSpheres::iterator iterSphere = collisionSpheres.begin();
		for(; iterSphere != collisionSpheres.end(); iterSphere++)
		{
			(*iterBox)->calculateInternals();
			(*iterSphere)->calculateInternals();
			usedContact += CollisionDetector::box_sphere(**iterBox, **iterSphere, &collisionData);
		}
	}

	//sphere-not box collisionModels
	CollisionSpheres::iterator iterSphere = collisionSpheres.begin();
	for(; iterSphere != collisionSpheres.end(); iterSphere++)
	{
		CollisionPlanes::iterator iterPlane= collisionPlanes.begin();
		for(; iterPlane != collisionPlanes.end(); iterPlane++)
		{
			(*iterSphere)->calculateInternals();
			usedContact += CollisionDetector::sphere_halfSpace(**iterSphere, **iterPlane, &collisionData);
		}

		CollisionSpheres::iterator iterSphere2 = collisionSpheres.begin();
		for(; iterSphere2 != collisionSpheres.end(); iterSphere2++)
		{
			if(*iterSphere == *iterSphere2) continue;
			(*iterSphere)->calculateInternals();
			(*iterSphere2)->calculateInternals();
			usedContact += CollisionDetector::sphere_sphere(**iterSphere, **iterSphere2, &collisionData);
		}
	}
	return usedContact;
}

void RPEngine::RPWorld::_collisionEventExcute(real duration)
{
	BodyCollisionEvents::iterator iter1 = bodyCollisionEvents.begin();
	CollisionNumberEvents::iterator iter2 = getCollisionNumberEvents.begin();
	CollisionWithEvents::iterator iter3 = getCollisionWithEvents.begin();
	CollisionWithArrayEvents::iterator iter4 = getCollisionWithArrayEvents.begin();

	for(; iter1 != bodyCollisionEvents.end(); iter1++)
		(*iter1)->execute(duration);
	
	for(; iter2 != getCollisionNumberEvents.end(); iter2++)
		(*iter2)->execute(duration);

	for(; iter3 != getCollisionWithEvents.end(); iter3++)
		(*iter3)->execute(duration);

	for(; iter4 != getCollisionWithArrayEvents.end(); iter4++)
		(*iter4)->execute(duration);
}

void RPWorld::integrate(real duration)
{
	RigidBodies::iterator body = bodies.begin();
#ifdef GET_OBJECTS_FORCE
	Vector3 maxLinearForce;
	Vector3 maxAngularForce;
#endif
	for(; body != bodies.end(); body++)
	{
		(*body)->integrate(duration);
#ifdef GET_OBJECTS_FORCE
		if((*body)->linearForce.magnitude() > maxLinearForce.magnitude())
			maxLinearForce = (*body)->linearForce;
		if((*body)->angularForce.magnitude() > maxAngularForce.magnitude())
			maxAngularForce = (*body)->angularForce;
#endif
	}
#ifdef GET_OBJECTS_FORCE
	BiggestLinearForceAtFrame = maxLinearForce;
	BiggestAngularForceAtFrame = maxAngularForce;

#endif
}

unsigned int RPWorld::generateContacts()
{
	if(collisionData.isFull()) return 0;
	unsigned int limit = maxContacts;

	for(ContactGenerators::iterator g = contactGenerators.begin();
		g != contactGenerators.end(); g++)
	{
		unsigned int used = (*g)->addContact(&collisionData, limit);
		//collisionData.addContacts(used); //count the used data
		limit -= used;

		if(limit <= 0) break;
	}
	
	return maxContacts - limit; // the number of contact used
}

/* body collision events */
//bodyCollision
BodyCollision *RPWorld::addBodyCollisionEvent(
	RigidBody *body1, 
	RigidBody *body2, 
	void(*callBack)() /*= NULL*/,
	real deltaVelocity,
	real eventCoolTime)
{
	BodyCollision *collision = new BodyCollision;
	collision->set(body1, body2, deltaVelocity, &collisionData, callBack, eventCoolTime);
	bodyCollisionEvents.push_back(collision);
	return collision;
}

bool RPEngine::RPWorld::removeBodyCollisionEvent(BodyCollision *collisionEvent)
{
	BodyCollisionEvents::iterator iter = bodyCollisionEvents.begin();
	for(; iter != bodyCollisionEvents.end(); iter++)
		if(collisionEvent == (*iter))
		{
			bodyCollisionEvents.erase(iter);
			return true;
		}
	return false;
}

void RPEngine::RPWorld::clearBodyCollisionEvent()
{
	BodyCollisionEvents::iterator iter = bodyCollisionEvents.begin();
	for(; iter != bodyCollisionEvents.end(); iter++)
		delete (*iter);
	bodyCollisionEvents.clear();
}

//getCollisionNumber
CollisionNumber *RPWorld::addCollisionNumberEvent(
	RigidBody *bodyTarget, 
	real deltaVelocity,
	real eventCoolTime)
{
	CollisionNumber *collision = new CollisionNumber;
	collision->set(bodyTarget, deltaVelocity, &collisionData, eventCoolTime);
	getCollisionNumberEvents.push_back(collision);

	return collision;
}

bool RPWorld::removeCollisionNumberEvent(CollisionNumber *collisionEvent)
{
	CollisionNumberEvents::iterator iter = getCollisionNumberEvents.begin();
	for(; iter != getCollisionNumberEvents.end(); iter++)
		if(collisionEvent == (*iter))
		{
			getCollisionNumberEvents.erase(iter);
			return true;
		}
	return false;
}

void RPEngine::RPWorld::clearCollisionNumberEvent()
{
	CollisionNumberEvents::iterator iter = getCollisionNumberEvents.begin();
	for(; iter != getCollisionNumberEvents.end(); iter++)
		delete (*iter);
	getCollisionNumberEvents.clear();
}

//getBodyCollisionWith
CollisionWith *RPWorld::addCollisionWithEvent(
	RigidBody *bodyTarget,
	real deltaVelocity,
	real eventCoolTime)
{
	CollisionWith *collision = new CollisionWith;
	collision->set(bodyTarget, deltaVelocity, &collisionData, eventCoolTime);
	getCollisionWithEvents.push_back(collision);
	return collision;
}

bool RPWorld::removeCollisionWithEvent(CollisionWith *collisionEvent)
{
	CollisionWithEvents::iterator iter = getCollisionWithEvents.begin();
	for(; iter != getCollisionWithEvents.end(); iter++)
		if(collisionEvent == (*iter))
		{
			getCollisionWithEvents.erase(iter);
			return true;
		}
	return false;
}

void RPEngine::RPWorld::clearCollisionWithEvent()
{
	CollisionWithEvents::iterator iter = getCollisionWithEvents.begin();
	for(; iter != getCollisionWithEvents.end(); iter++)
		delete (*iter);
	getCollisionWithEvents.clear();
}

//getCollisionWithArray
CollisionWithArray * RPWorld::addCollisionWithArrayEvent(
	RigidBody *bodyTarget, 
	int limit,
	real deltaVelocity,
	real eventCoolTime)
{
	CollisionWithArray *collision = new CollisionWithArray;
	collision->set(bodyTarget, deltaVelocity, limit, &collisionData, eventCoolTime);
	getCollisionWithArrayEvents.push_back(collision);
	return collision;
}

bool RPWorld::removeCollisionWithArrayEvent(CollisionWithArray *collisionEvent)
{
	CollisionWithArrayEvents::iterator iter = getCollisionWithArrayEvents.begin();
	for(; iter != getCollisionWithArrayEvents.end(); iter++)
		if(collisionEvent == (*iter))
		{
			getCollisionWithArrayEvents.erase(iter);
			return true;
		}
	return false;
}

void RPEngine::RPWorld::clearCollisionWithArrayEvent()
{
	CollisionWithArrayEvents::iterator iter = getCollisionWithArrayEvents.begin();
	for(; iter != getCollisionWithArrayEvents.end(); iter++)
		delete (*iter);

	getCollisionWithArrayEvents.clear();
}

/* utilities */

void RPEngine::RPWorld::UndoContactGenerate(unsigned int size)
{
	if(contactGenerators.size() < size) 
	{
		std::cout << "contactGenerator가 가진 양보다 더 많은 것을 지우려 시도했습니다.\n";
		return; //assert
	}
	ContactGenerators::iterator iter = contactGenerators.end();
	iter--; //end-1 idx
	for(unsigned int i = 0; i < size; iter--)
	{
		contactGenerators.erase(iter);
	}
}

void RPEngine::RPWorld::UndoForceGenerate(unsigned int size)
{
	if(registry.registrations.size() < size) 
	{
		std::cout << "registry가 가진 양보다 더 많은 것을 지우려 시도했습니다.\n";
		return; //assert
	}
	ForceRegistry::Registry::iterator iter = registry.registrations.end();
	iter--; //end-1 idx
	for(unsigned int i = 0; i < size; iter--)
	{
		registry.registrations.erase(iter);
	}
}

void RPEngine::RPWorld::clearData()
{
	bodies.clear();
	registry.clear();

	contactGenerators.clear();

	collisionBoxes.clear();
	collisionSpheres.clear();
	collisionPlanes.clear();

	bodyCollisionEvents.clear();
	getCollisionNumberEvents.clear();
	getCollisionWithEvents.clear();
	getCollisionWithArrayEvents.clear();
}

#endif
