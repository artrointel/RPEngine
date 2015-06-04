#ifndef _STATIC_JOINT_
#define _STATIC_JOINT_

#include<iostream>
#include "staticJoint.h"

using namespace RPEngine;

int StaticJoint::addContact(CollisionData *data, int limit) const
{
	if (data->isFull()) return 0;
	Contact* contact = data->contacts;
	// The position of each connection point in WCS
	Vector3 a_pos_world = body[0]->getPointInWorldSpace(position[0]);
	Vector3 b_pos_world;
	if(body[1])
		b_pos_world = body[1]->getPointInWorldSpace(position[1]);
	else //body is NULL, then use the position[1] point be in WCS
	{
		b_pos_world = position[1];
		contact->body[1] = NULL;
	}

	// Calculate the length of the joint
	Vector3 a_to_b = b_pos_world - a_pos_world;
	Vector3 relativePositionNormal = a_to_b;
	relativePositionNormal.normalize();
	real length = a_to_b.magnitude();

	// Check if it is violated
	if (real_abs(length-staticDistance) > epsilonDistance)
	{
		contact->body[0] = body[0];
		contact->body[1] = body[1];
		contact->contactNormal = relativePositionNormal;
		contact->contactPoint = (a_pos_world + b_pos_world) * 0.5f;
		contact->penetration = real_abs(length-staticDistance) - epsilonDistance;
		contact->friction = 1.0f;
		contact->restitution = 0;
		data->addContacts(1);
		return 1;
	}

	return 0;
}

void StaticJoint::set(RigidBody *bodyA, const Vector3& a_pos,
	RigidBody *bodyB, const Vector3& b_pos,
	real _epsilonDistance, real _staticDistance)
{
	body[0] = bodyA;
	body[1] = bodyB;

	position[0] = a_pos;
	position[1] = b_pos;

	epsilonDistance = _epsilonDistance;
	staticDistance = _staticDistance;
}

#endif
