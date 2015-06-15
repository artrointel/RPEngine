#ifndef _CONTACTS_ 
#define _CONTACTS_ 

#include<assert.h>
#include<iostream>
#include"contacts.h"

using namespace RPEngine;

#define DEBUG

/* Contact */
void Contact::setBodyData(RigidBody *body1, RigidBody *body2, real friction, real restitution)
{
	Contact::body[0] = body1;
	Contact::body[1] = body2;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

void Contact::calculateContactBasis()
{
	Vector3 contactTangent[2];

	/* check the contactNormal axis */
	//it is nearby x-axis
	if(real_abs(contactNormal.x) > real_abs(contactNormal.y))
	{
		//normalize
		const real s = static_cast<real>(1.0f) / 
			real_sqrt(contactNormal.z*contactNormal.z +
					  contactNormal.x*contactNormal.x);

		//y-axis is at right angle to WCS y-axis
		contactTangent[0].x = contactNormal.z*s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x*s;

		//z-axis right angle to the others
		contactTangent[1].x = contactNormal.y*contactTangent[0].x;
		contactTangent[1].y = contactNormal.z*contactTangent[0].x - 
			contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = -contactNormal.x*s;
	}
	//it is nearby y-axis
	else
	{
		//normalize
		const real s = static_cast<real>(1.0f) / 
			real_sqrt(contactNormal.z*contactNormal.z +
			contactNormal.y*contactNormal.y);

		//y-axis is at right angle to WCS x-axis
		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z*s;
		contactTangent[0].z = contactNormal.y*s;

		//z-axis right angle to the others
		contactTangent[1].x = contactNormal.y*contactTangent[0].z -
			contactNormal.z*contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = contactNormal.x*contactTangent[0].y;
	}

	//make contact coordinate transform matrix
	contactToWorld.setComponents(contactNormal, contactTangent[0], contactTangent[1]);

}

void Contact::calculateDeltaVelocity(Matrix4 *inverseInertiaTensor)
{
	//WCS
	Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal; // torque per unit impulse
	deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld); // rotation(angular velocity) per unit impulse
	deltaVelWorld = deltaVelWorld % relativeContactPosition[0]; //linear velocity per unit impulse 

	real deltaVelocity = deltaVelWorld * contactNormal; // convert linear velocity per unit impulse WCS to CCS

	deltaVelocity += body[0]->getInverseMass();

	if(body[1])
	{
		Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal; // torque per unit impulse
		deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld); // rotation(angular velocity) per unit impulse
		deltaVelWorld = deltaVelWorld % relativeContactPosition[1]; //linear velocity per unit impulse 
		
		deltaVelocity += deltaVelWorld * contactNormal;
		deltaVelocity += body[1]->getInverseMass();
	}
}

void Contact::calculateInternals(real duration)
{
	if(!body[0]) swapBodies();
#ifdef DEBUG
	if(!body[0])
	{
		std::cout << "충돌 자료가 잘못 된 값을 가지고 있으므로 종료합니다.\n";
		std::cout << "충돌 데이터의 두 강체가 메모리에 존재하지 않습니다.\n";
		std::cout << &body[0] << ", " << &body[1] << std::endl;
	}
#endif
	assert(body[0]);
	//contact basis ready
	calculateContactBasis();
	
	//relative contact position ready
	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if(body[1])
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();
	
	//contact velocity ready
	contactVelocity = calculateLocalVelocity(0, duration);
	if(body[1])
		contactVelocity -= calculateLocalVelocity(1, duration);

	calculateDesiredDeltaVelocity(duration);
}

void Contact::swapBodies()
{
	contactNormal *= -1;
	RigidBody *temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

Vector3 Contact::calculateLocalVelocity(int index, real duration)
{
	RigidBody *pBody = body[index];

	//angular velocity
	Vector3 velocity = pBody->getRotation() % relativeContactPosition[index];
	//linear velocity
	velocity += pBody->getVelocity();
	
	//WCS to CCS
	Vector3 contactVelocity = contactToWorld.transformTranspose(velocity);
	//Vector3 accVelocity = pBody->getLastFrameAcceleration() * duration;

	return contactVelocity;
}

//if velocity by impulse is too small, then we ignore restitution
//but it can cause vibration, so we lower the restitution 
#define INSENSITIVITY 0.15f 
void Contact::calculateDesiredDeltaVelocity(real duration)
{
	// it might be sort of multiplication of duration
	const static real velocityLimit = static_cast<real>(INSENSITIVITY);
	real lastVelocityFromAcc = 0; // by lastframe accel
	real thisRestitution = restitution;

	if (real_abs(contactVelocity.x) < velocityLimit)
		thisRestitution = restitution * (contactVelocity.x / velocityLimit); //lower

	lastVelocityFromAcc += 
		body[0]->getLastFrameAcceleration() * duration * contactNormal;

	if (body[1])
		lastVelocityFromAcc -=
			body[1]->getLastFrameAcceleration() * duration * contactNormal;

	
	// combine the bounce velocity with the removed acceleration velocity.
	desiredDeltaVelocity =
		-contactVelocity.x
		-thisRestitution * (contactVelocity.x - lastVelocityFromAcc);
	//this can remove some linear vibration. but not all yet.

}

//refactoring required function
void Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration)
{
	static const real angularLimit = (real)0.2f;
	real angularMove[2];
	real linearMove[2];

	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];

	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		Matrix3 inverseInertiaTensor;
		body[i]->fillInverseInertiaTensorWorld(&inverseInertiaTensor);

		//an unit of impulsive torque
		Vector3 angularInertiaWorld =
			relativeContactPosition[i] % contactNormal;
		//rotation per an unit of impulsive torque
		angularInertiaWorld =
			inverseInertiaTensor.transform(angularInertiaWorld);
		//rotation at contact point in WCS
		angularInertiaWorld =
			angularInertiaWorld % relativeContactPosition[i];
		//WCS to CCS
		angularInertia[i] =
			angularInertiaWorld * contactNormal;

		//linear velocity per an unit of impulse
		linearInertia[i] = body[i]->getInverseMass();

		totalInertia += linearInertia[i] + angularInertia[i];
	}

	//position application
	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		real sign = (i == 0) ? 1 : -1;
		angularMove[i] = sign*penetration *(angularInertia[i]/totalInertia);
		linearMove[i] = sign*penetration *(linearInertia[i]/totalInertia);

		//to avoid too great angular projections, limit the movement
		Vector3 projection = relativeContactPosition[i];
		projection.addScaledVector(contactNormal,
			-relativeContactPosition[i].scalarProduct(contactNormal));
		real maxMagnitude = angularLimit * projection.magnitude();

		/////////////
		real totalMove = angularMove[i] + linearMove[i];
		if (angularMove[i] < -maxMagnitude)
			angularMove[i] = -maxMagnitude;
		else if (angularMove[i] > maxMagnitude)
			angularMove[i] = maxMagnitude;
		linearMove[i] = totalMove - angularMove[i];
		
		//calculate position Change at linear, angular
		if (IsEqual(angularMove[i], 0)) 
			angularChange[i].clear();
		else
		{
			Vector3 rotateDirection = relativeContactPosition[i].vectorProduct(contactNormal);

			Matrix3 inverseInertiaTensor;
			body[i]->fillInverseInertiaTensorWorld(&inverseInertiaTensor);

			// Work out the direction we'd need to rotate to achieve that
			angularChange[i] = 
				inverseInertiaTensor.transform(rotateDirection) * (angularMove[i] / angularInertia[i]);
		}
		linearChange[i] = contactNormal * linearMove[i];
		
		//apply position, orientation change
		body[i]->addScaledVectorPosition(contactNormal, linearMove[i]);
		body[i]->addScaledVectorOrientation(angularChange[i], 1.f);
		//
		//if (!body[i]->getAwake()) body[i]->calculateDerivedData();
	}
}

//다시보기
void Contact::applyVelocityChange(Vector3 velocityChange[2],
	Vector3 rotationChange[2])
{
	// world coordinates.
	Matrix3 inverseInertiaTensor[2];
	body[0]->fillInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
	if (body[1])
		body[1]->fillInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

	Vector3 impulseContact;
	
	if (IsEqual(friction, 0.0))
	{
		impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
	}

	// CCS impulse to WCS
	Vector3 impulse = contactToWorld.transform(impulseContact);

	//apply velocity & rotation
	Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
	rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
	velocityChange[0].clear();
	velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());

	body[0]->addVelocity(velocityChange[0]);
	body[0]->addRotation(rotationChange[0]);

	if (body[1])
	{
		Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
		rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
		velocityChange[1].clear();
		velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());

		body[1]->addVelocity(velocityChange[1]);
		body[1]->addRotation(rotationChange[1]);
	}
}

/* Contact Resolver */
ContactResolver::ContactResolver(int iterations, 
	real _velocityEpsilon, 
	real _positionEpsilon)
{
	setIterations(iterations);
	setEpsilon(_velocityEpsilon, _positionEpsilon);
}

ContactResolver::ContactResolver(int _velocityIterations,
	int _positionIterations,
	real _velocityEpsilon,
	real _positionEpsilon)
{
	setIterations(_velocityIterations, _positionIterations);
	setEpsilon(_velocityEpsilon, _positionEpsilon);
}

void ContactResolver::resolveContacts(Contact *contactArray, 
										unsigned int numContacts, 
										real duration)
{
	if(numContacts == 0) return;

	prepareContacts(contactArray, numContacts, duration);

	//resolve penetration by modifying the position
	adjustPositions(contactArray, numContacts, duration);

	//resolve impulse by velocity changes
	adjustVelocities(contactArray, numContacts, duration);
}

//get, set
void ContactResolver::setIterations(int iterations)
{
	velocityIterations = positionIterations = iterations;
}

void ContactResolver::setIterations(int _velocityIterations, int _positionIterations)
{
	velocityIterations = _velocityIterations;
	positionIterations = _positionIterations;
}

void ContactResolver::setEpsilon(real _velocityEpsilon, real _positionEpsilon)
{
	velocityEpsilon = _velocityEpsilon;
	positionEpsilon = _positionEpsilon;
}

bool ContactResolver::isValid()
{
	return (velocityIterations > 0) &&
		(positionIterations > 0) &&
		(positionEpsilon >= 0.0f) &&
		(positionEpsilon >= 0.0f);
}

void ContactResolver::prepareContacts(Contact *contacts, //contactArray
										int numContacts,
										real duration)
{
	Contact *lastContact = contacts + numContacts;

	for(Contact *contact = contacts; contact < lastContact; contact++)
	{
		contact->calculateInternals(duration);
	}
}

void ContactResolver::adjustPositions(Contact *contacts, 
										int numContacts, 
										real duration)
{
	int index;
	Vector3 linearChange[2], angularChange[2];
	real max;
	Vector3 deltaPosition;

	positionIterationsUsed = 0;
	while(positionIterationsUsed < positionIterations)
	{
		// biggest penetration depth search
		max = positionEpsilon;
		index = numContacts;
		for(int i = 0; i < numContacts; i++)
		{
			if(contacts[i].penetration > max)
			{
				max = contacts[i].penetration;
				index = i;
			}
		}
		if(index == numContacts) break;

		//contacts[index].matchAwakeState();

		//resolve penetration
		contacts[index].applyPositionChange(linearChange, angularChange, max);
		
		//again this action may have changed the penetration of other bodies.
		//so we update contacts
		for(int i = 0; i < numContacts; i++)
			for(int j = 0; j < 2; j++) if(contacts[i].body[j])
				for(int k = 0; k < 2; k++)
					if(contacts[i].body[j] == contacts[index].body[k])
					{
						deltaPosition = linearChange[k] + 
							angularChange[k].vectorProduct(contacts[i].relativeContactPosition[j]);
						contacts[i].penetration += 
							deltaPosition.scalarProduct(contacts[i].contactNormal)*(j? 1:-1);
					}
		positionIterationsUsed++;
	}
}

void ContactResolver::adjustVelocities(Contact *contacts, 
	int numContacts, 
	real duration)
{
	Vector3 velocityChange[2], rotationChange[2];
	Vector3 deltaVel;

	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		real max = velocityEpsilon;
		int index = numContacts;
		for (int i = 0; i < numContacts; i++)
		{
			if (contacts[i].desiredDeltaVelocity > max)
			{
				max = contacts[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts) break;

		contacts[index].applyVelocityChange(velocityChange, rotationChange);

		// velocities need recomputing.
		for (int i = 0; i < numContacts; i++)
			for (int j = 0; j < 2; j++) if (contacts[i].body[j])
				for (unsigned k = 0; k < 2; k++)// check newly resolved contact
					if (contacts[i].body[j] == contacts[index].body[k])
					{
						deltaVel = velocityChange[k] +
							rotationChange[k].vectorProduct(contacts[i].relativeContactPosition[j]);

						contacts[i].contactVelocity +=
							contacts[i].contactToWorld.transformTranspose(deltaVel)*(j?-1:1);

						contacts[i].calculateDesiredDeltaVelocity(duration);
					}
		velocityIterationsUsed++;
	}
}

//
inline Vector3 Contact::calculateFrictionlessImpulse(Matrix3 * inverseInertiaTensor)
{
	Vector3 impulseContact;

	Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
	deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
	deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

	real deltaVelocity = deltaVelWorld * contactNormal;

	deltaVelocity += body[0]->getInverseMass();

	if (body[1])
	{
		Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
		deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld % relativeContactPosition[1];

		deltaVelocity += deltaVelWorld * contactNormal;

		deltaVelocity += body[1]->getInverseMass();
	}

	impulseContact.x = desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0;
	impulseContact.z = 0;
	return impulseContact;
}

//////// 마찰력적용한 계산은 긁어온 것입니다
//책에 설명과 함께 코드가 있는데 나중에 책보고 다시 구현
inline
Vector3 Contact::calculateFrictionImpulse(Matrix3 * inverseInertiaTensor)
{
	Vector3 impulseContact;
	real inverseMass = body[0]->getInverseMass();

	// The equivalent of a cross product in matrices is multiplication
	// by a skew symmetric matrix - we build the matrix for converting
	// between linear and angular quantities.
	Matrix3 impulseToTorque;
	impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

	// Build the matrix to convert contact impulse to change in velocity
	// in world coordinates.
	Matrix3 deltaVelWorld = impulseToTorque;
	deltaVelWorld *= inverseInertiaTensor[0];
	deltaVelWorld *= impulseToTorque;
	deltaVelWorld *= -1;

	// Check if we need to add body two's data
	if (body[1])
	{
		// Set the cross product matrix
		impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);

		// Calculate the velocity change matrix
		Matrix3 deltaVelWorld2 = impulseToTorque;
		deltaVelWorld2 *= inverseInertiaTensor[1];
		deltaVelWorld2 *= impulseToTorque;
		deltaVelWorld2 *= -1;

		// Add to the total delta velocity.
		deltaVelWorld = deltaVelWorld + deltaVelWorld2;

		// Add to the inverse mass
		inverseMass += body[1]->getInverseMass();
	}

	// Do a change of basis to convert into contact coordinates.
	Matrix3 deltaVelocity = contactToWorld.transpose();
	deltaVelocity *= deltaVelWorld;
	deltaVelocity *= contactToWorld;

	// Add in the linear velocity change
	deltaVelocity.data[0] += inverseMass;
	deltaVelocity.data[4] += inverseMass;
	deltaVelocity.data[8] += inverseMass;

	// Invert to get the impulse needed per unit velocity
	Matrix3 impulseMatrix = deltaVelocity.inverse();

	// Find the target velocities to kill
	Vector3 velKill(desiredDeltaVelocity,
		-contactVelocity.y,
		-contactVelocity.z);

	// Find the impulse to kill target velocities
	impulseContact = impulseMatrix.transform(velKill);

	// Check for exceeding friction
	real planarImpulse = real_sqrt(
		impulseContact.y*impulseContact.y +
		impulseContact.z*impulseContact.z
		);
	if (planarImpulse > impulseContact.x * friction)
	{
		// We need to use dynamic friction
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;

		impulseContact.x = deltaVelocity.data[0] +
			deltaVelocity.data[1]*friction*impulseContact.y +
			deltaVelocity.data[2]*friction*impulseContact.z;
		impulseContact.x = desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= friction * impulseContact.x;
		impulseContact.z *= friction * impulseContact.x;
	}
	return impulseContact;
}

#endif

/*
Matrix3 inverseInertiaTensor;
body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor) //fill로 바꿔버리자.
// 단위 돌림 충격량 계산 
Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
// 단위 돌림 충격량 당 각속도 변화량 계산 
angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
// 충돌지점의 각속도에 의한 속도 벡터를 WCS에서의 계산 
angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
// WCS에서 계산한 각속도를 CCS의 각속도로 변환(CCS 단위충격량 당 각속도임)
angularInertia[i] = angularInertiaWorld * contactNormal;

linearInertia[i] = body[i]->getInverseMass();
totalInertia += linearInertia[i] + angularInertia[i]; //사실 상angular는 관성모멘트
*/
