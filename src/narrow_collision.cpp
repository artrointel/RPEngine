#ifndef _NARROW_COLLISION_
#define _NARROW_COLLISION_
#include<stdio.h> // NULL
#include<iostream>
#include<iomanip>
#include "narrow_collision.h"

//#define DEBUG

using namespace RPEngine;

void CollisionPrimitive::calculateInternals() //calculate matrix gets body's transform*offset for collision detect
{
	transform = body->getTransform() * offset;
}


Vector3 CollisionPrimitive::getAxis(unsigned int index) const
{
	return transform.getAxisVector(index);
}

unsigned int CollisionDetector::sphere_sphere(const CollisionSphere &sphere1, 
	const CollisionSphere &sphere2, 
	CollisionData *data)
{
	if(data->contactsLeft <= 0) return 0;

	//get sphere's position
	Vector3 position1 = sphere1.getAxis(3);
	Vector3 position2 = sphere2.getAxis(3);

	//find the object vector
	Vector3 centerVector = position1 - position2;
	real size = centerVector.magnitude();

	//is collision?
	if(size <= 0.0f || size >= (sphere1.radius + sphere2.radius))
		return 0;

	//normalize centerVector
	Vector3 normalVector = centerVector*(static_cast<real>(1.0)/size); //i fixed.

	//fillContact
	Contact *contact = data->contacts;
	contact->contactNormal = normalVector;
	contact->contactPoint = position1 + centerVector*static_cast<real>(0.5); // it might be 'position2 -' i think
	contact->penetration = (sphere1.radius + sphere2.radius - size); //penetration at that frame
	contact->setBodyData(sphere1.body, sphere2.body, 
						 data->friction, data->restitution);
	//set friction, restitution...
	data->addContacts(1);
	return 1;
}

unsigned int CollisionDetector::sphere_halfSpace(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data)
{
	if(data->isFull()) return 0; // considering making check function.
	
	Vector3 position = sphere.getAxis(3);

	real ballDistance = plane.direction * position - sphere.radius - plane.offset; //how??

	if(ballDistance >= 0) return 0;

	//fillContact
	Contact *contact = data->contacts;
	contact->contactNormal = plane.direction;
	contact->penetration = -ballDistance; // penetration > 0
	contact->contactPoint = position - plane.direction * (ballDistance + sphere.radius);
	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);
	
	data->addContacts(1);
	return 1;
}

unsigned int CollisionDetector::sphere_truePlane(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data)
{
	if(data->contactsLeft <= 0) return 0;

	Vector3 position = sphere.getAxis(3);
	real centerDistance = plane.direction * position - plane.offset;
	if(centerDistance*centerDistance > sphere.radius*sphere.radius)
		return 0;

	Vector3 normal = plane.direction;
	real penetration = -centerDistance;
	if(centerDistance < 0)
	{
		normal *= -1;
		penetration = -penetration;
	}
	penetration += sphere.radius;

	//fillContact
	Contact *contact = data->contacts;
	contact->contactNormal = plane.direction;
	contact->penetration = penetration; // penetration > 0
	contact->contactPoint = position - plane.direction*centerDistance;
	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

unsigned int CollisionDetector::box_halfSpace(const CollisionBox &box,
	const CollisionPlane &plane,
	CollisionData *data)
{
	if (data->isFull()) return 0;

	// Check for intersection
	//if (!IntersectionTests::boxAndHalfSpace(box, plane))
	//	return 0;

	static real halfVertex[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
	{1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

	Contact* contact = data->contacts;
	int contactsUsed = 0;
	for (int i = 0; i < 8; i++) 
	{
		// the position of each vertex
		Vector3 vertexPos(halfVertex[i][0], halfVertex[i][1], halfVertex[i][2]);
		vertexPos.componentProductUpdate(box.halfSize); //
		vertexPos = box.transform.transform(vertexPos); //offset

		// calculate the distance from the plane
		real vertexDistance = vertexPos * plane.direction;

		// fillContact
		if (vertexDistance <= plane.offset)
		{
			contact->contactNormal = plane.direction;
			contact->penetration = plane.offset - vertexDistance;
			contact->contactPoint = plane.direction*(vertexDistance-plane.offset) + vertexPos;

			contact->setBodyData(box.body, NULL,
				data->friction, data->restitution);
			contact++;
			contactsUsed++;
			if (contactsUsed == data->contactsLeft) return contactsUsed;
		}
	}

	data->addContacts(contactsUsed);
	return contactsUsed;
}

unsigned int CollisionDetector::box_sphere(const CollisionBox &box,
	const CollisionSphere &sphere,
	CollisionData *data)
{
	//transform center of the sphere into box coordinates.
	Vector3 sphere_center = sphere.getAxis(3);
	Vector3 relCenter = box.transform.transformInverse(sphere_center);

	//early check;is contact? then we can exclude
	if(real_abs(relCenter.x) - sphere.radius > box.halfSize.x ||
	   real_abs(relCenter.y) - sphere.radius > box.halfSize.y ||
	   real_abs(relCenter.z) - sphere.radius > box.halfSize.z)
	{
		return 0;
	}

	//find closest point in the box
	Vector3 closestPoint(0,0,0);
	real distance = relCenter.x;
	if(distance > box.halfSize.x)  distance = box.halfSize.x;
	if(distance < -box.halfSize.x) distance = -box.halfSize.x;
	closestPoint.x = distance;

	distance = relCenter.y;
	if(distance > box.halfSize.y)  distance = box.halfSize.y;
	if(distance < -box.halfSize.y) distance = -box.halfSize.y;
	closestPoint.y = distance;

	distance = relCenter.z;
	if(distance > box.halfSize.z)  distance = box.halfSize.z;
	if(distance < -box.halfSize.z) distance = -box.halfSize.z;
	closestPoint.z = distance;

	//is contact?
	distance = (closestPoint - relCenter).squareMagnitude();
	if(distance > sphere.radius*sphere.radius) return 0;

	Vector3 closestPtWorld = box.transform.transform(closestPoint);

	//fillContact
	Contact *contact = data->contacts;
	contact->contactNormal = (closestPtWorld - sphere_center);
	contact->contactNormal.normalize();
	contact->contactPoint = closestPtWorld;
	contact->penetration = sphere.radius - real_sqrt(distance);
	contact->setBodyData(box.body, sphere.body, data->friction, data->restitution);
	
	data->addContacts(1);
	return 1;
}


/* for SAT functions */

//project the box to axis!
static inline real transformToAxis(const CollisionBox &box,
	const Vector3 &axis) //return projection size
{
	return
		box.halfSize.x * real_abs(axis * box.getAxis(0)) + //project the axis at x
		box.halfSize.y * real_abs(axis * box.getAxis(1)) + //project the axis at y
		box.halfSize.z * real_abs(axis * box.getAxis(2)); //project the axis at z
}

static inline real penetrationOnAxis(const CollisionBox &box1, 
	const CollisionBox &box2, 
	const Vector3 &axis, 
	const Vector3 &toCenter) //toCenter : first->second
{
	real box1Projection = transformToAxis(box1, axis);
	real box2Projection = transformToAxis(box2, axis);
	real distance = real_abs(toCenter * axis);

	return box1Projection + box2Projection - distance;
}

static inline bool tryAxis(
	const CollisionBox &box1,
	const CollisionBox &box2,
	Vector3 axis,
	const Vector3& toCenter,
	unsigned int index,
	real& smallestPenetration,
	unsigned int &smallestCase)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squareMagnitude() < 0.0001) return true;
	axis.normalize();

	real penetration = penetrationOnAxis(box1, box2, axis, toCenter);
	if (penetration < 0) return false;
	if (penetration < smallestPenetration) 
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

// box2 vertex, box1 face collision
void fillPointFace_box_box(
	const CollisionBox &box1,
	const CollisionBox &box2,
	const Vector3 &toCenter,
	CollisionData *data,
	int smallestCase,
	real smallestPenetrate)
{
	Contact* contact = data->contacts;

	//which face is in contact?
	Vector3 normalVector = box1.getAxis(smallestCase); // face normal vector
	if (normalVector * toCenter > 0)
		normalVector = normalVector * -1.0f;

	//which vertex is in contact at box2 coordinate? 
	Vector3 vertex = box2.halfSize;
	if(box2.getAxis(0) * normalVector < 0) vertex.x = -vertex.x;
	if(box2.getAxis(1) * normalVector < 0) vertex.y = -vertex.y;
	if(box2.getAxis(2) * normalVector < 0) vertex.z = -vertex.z;

	//convert to work coordinate
	vertex = box2.transform * vertex; ///////////

	// Create the contact data
	contact->contactNormal = normalVector;
	contact->penetration = smallestPenetrate;
	contact->contactPoint = vertex;
	contact->setBodyData(box1.body, box2.body,
		data->friction, data->restitution);
}
//////////
static inline Vector3 contactPoint(
	Vector3 &pOne, //const
	Vector3 &dOne, //const
	real oneSize,
	Vector3 &pTwo, //const
	Vector3 &dTwo, //const
	real twoSize,

	// If this is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	real denom, mua, mub;

	smOne = dOne.squareMagnitude();
	smTwo = dTwo.squareMagnitude();
	dpOneTwo = dTwo * dOne;

	toSt = pOne - pTwo;
	dpStaOne = dOne * toSt;
	dpStaTwo = dTwo * toSt;

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (real_abs(denom) < 0.0001f) {
		return useOne? pOne:pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne?pOne:pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

//it might be checked performance, I think
#define CHECK_OVERLAP(axis, index) \
	if (!tryAxis(box1, box2, (axis), toCenter, (index), smallestPenetration, smallestCase)) return 0;
unsigned int CollisionDetector::box_box(const CollisionBox &box1,
										const CollisionBox &box2,
										CollisionData *data)
{
	Vector3 toCenter = box2.getAxis(3) - box1.getAxis(3);
	real smallestPenetration = REAL_MAX;
	unsigned int smallestCase = 0xffffff;

	//hard typed for performance these collision checking at 15 axis
	
	//cache for axis
	Vector3 box1_x = box1.getAxis(0);
	Vector3 box1_y = box1.getAxis(1);
	Vector3 box1_z = box1.getAxis(2);

	Vector3 box2_x = box2.getAxis(0);
	Vector3 box2_y = box2.getAxis(1);
	Vector3 box2_z = box2.getAxis(2);

	//6 face test
	CHECK_OVERLAP(box1_x, 0);
	CHECK_OVERLAP(box1_y, 1);
	CHECK_OVERLAP(box1_z, 2);

	CHECK_OVERLAP(box2_x, 3);
	CHECK_OVERLAP(box2_y, 4);
	CHECK_OVERLAP(box2_z, 5);
	int smallestCaseForRecompute = smallestCase;

	//9 edges test
	CHECK_OVERLAP(box1_x % box2_x, 6);
	CHECK_OVERLAP(box1_x % box2_y, 7);
	CHECK_OVERLAP(box1_x % box2_z, 8);

	CHECK_OVERLAP(box1_y % box2_x, 9);
	CHECK_OVERLAP(box1_y % box2_y, 10);
	CHECK_OVERLAP(box1_y % box2_z, 11);

	CHECK_OVERLAP(box1_z % box2_x, 12);
	CHECK_OVERLAP(box1_z % box2_y, 13);
	CHECK_OVERLAP(box1_z % box2_z, 14);
#ifdef DEBUG
	if(smallestCase == 0xffffff)
	{
		std::cout.setf(std::ios::showpoint);
		std::cout << "두 박스 충돌 데이터의 충돌 거리 검출에서 최단거리를 찾을 수 없었습니다.\n"
		<< "-- 충돌 데이터 --\n"
		<< "-박스1-\n"
		<< "크기:" << box1.halfSize.x*2 << ","<< box1.halfSize.y*2 <<"," << box1.halfSize.z*2 << std::endl
		<< "좌표:" << box1.body->getPosition().x << "," << box1.body->getPosition().y << "," << box1.body->getPosition().z << std::endl
		<< "속도:" << box1.body->getVelocity().x << "," << box1.body->getVelocity().y << "," << box1.body->getVelocity().z << std::endl
		<< "-박스2-\n"
		<< "크기:" << box2.halfSize.x*2 << ","<< box2.halfSize.y*2 <<"," << box2.halfSize.z*2 << std::endl
		<< "좌표:" << box2.body->getPosition().x << "," << box2.body->getPosition().y << "," << box2.body->getPosition().z << std::endl
		<< "속도:" << box2.body->getVelocity().x << "," << box2.body->getVelocity().y << "," << box2.body->getVelocity().z << std::endl
		<< std::endl;
		printf("박스1과 박스2의 메모리 주소:%p, %p\n",&box1.body, &box2.body);
		printf("충돌데이터의 메모리 주소:%p\n", &data);
	}
#endif
	assert(smallestCase != 0xffffff);

	// contact based on face axis
	if(smallestCase < 3) //box1 face, box2 vertex collision
	{
		fillPointFace_box_box(box1,box2, toCenter, 
							  data, smallestCase, smallestPenetration);
		data->addContacts(1);
		return 1;
	}
	else if(smallestCase < 6) //box1 vertex, box2 face collision
	{
		fillPointFace_box_box(box2, box1, toCenter*(-1.f), 
			data, smallestCase-3, smallestPenetration);
		data->addContacts(1);
		return 1;
	}
	else //edge-edge collision
	{
		//which axes?
		smallestCase -= 6;
		int box1AxisIndex = smallestCase / 3;
		int box2AxisIndex = smallestCase % 3;
		Vector3 box1Axis = box1.getAxis(box1AxisIndex);
		Vector3 box2Axis = box2.getAxis(box2AxisIndex);
		Vector3 axis = box1Axis % box2Axis;
		axis.normalize();

		// the axis should point from box1 to box2.
		if (axis * toCenter > 0) axis = axis * -1.0f;

		//find which of the 4 for each object. determine each of the other axes is closest.
		Vector3 ptOnEdgeOne = box1.halfSize;
		Vector3 ptOnEdgeTwo = box2.halfSize;
		for (int i = 0; i < 3; i++)
		{
			if (i == box1AxisIndex) ptOnEdgeOne[i] = 0;
			else if (box1.getAxis(i) * axis > 0) ptOnEdgeOne[i] = -ptOnEdgeOne[i];

			if (i == box2AxisIndex) ptOnEdgeTwo[i] = 0;
			else if (box2.getAxis(i) * axis < 0) ptOnEdgeTwo[i] = -ptOnEdgeTwo[i];
		}

		//move the point into WCS
		ptOnEdgeOne = box1.transform * ptOnEdgeOne;
		ptOnEdgeTwo = box1.transform * ptOnEdgeTwo;

		Vector3 vertex = contactPoint(
			ptOnEdgeOne, box1Axis, box1.halfSize[box1AxisIndex],
			ptOnEdgeTwo, box2Axis, box1.halfSize[box2AxisIndex],
			smallestCaseForRecompute > 2);

		//fillContact.
		Contact* contact = data->contacts;

		contact->penetration = smallestPenetration;
		contact->contactNormal = axis;
		contact->contactPoint = vertex;
		contact->setBodyData(box1.body, box2.body,
			data->friction, data->restitution);
		data->addContacts(1);
		return 1;
	}
	return 0;
}

/* search collision from collision data */

real GetClosingVelocity(
	Vector3 &contactVector, 
	RigidBody *contactBody1,
	RigidBody *contactBody2)
{
	Vector3 closingVector;
	if(contactBody1 == NULL)
	{
		closingVector = contactBody2->getVelocity();
	}
	else if(contactBody2 == NULL)
		closingVector = contactBody1->getVelocity();
	else
	{
		closingVector = 
			contactBody1->getVelocity() - contactBody2->getVelocity();
	}

	real closingVelocity  = real_abs(closingVector * contactVector);
	if(closingVelocity < 0) closingVelocity *= -1;
	return closingVelocity;
}

#define GET_CLOSING_VELOCITY(contactVector, contactBody1, contactBody2) \
	GetClosingVelocity((contactVector), (contactBody1), (contactBody2));

bool CollisionDetector::bodyCollision(
	RigidBody *body1, RigidBody *body2, real deltaVelocity, 
	CollisionData *_data, void(*callBack)() /*= NULL*/)
{
	if(!body1) return false;
	if(!body2) return false;
	for(int i = 0; i < _data->contactCount; i++)
	{
		RigidBody *contactBody1 = _data->contactArray[i].body[0];
		RigidBody *contactBody2 = _data->contactArray[i].body[1];
		if(
			(contactBody1 == body1 &&
		     contactBody2 == body2) ||
		    (contactBody1 == body2 &&
		     contactBody2 == body1)
		  )
		{
			if(callBack != NULL) (*callBack)();
			if(IsEqual(deltaVelocity, 0)) return true;

			Vector3 contactVector = _data->contactArray[i].contactNormal;
			real closingVelocity = GET_CLOSING_VELOCITY(
				contactVector, contactBody1, contactBody2);
			// colliding velocity
			if(deltaVelocity < closingVelocity)
				return true;
			else
				return false;
		}
	}
	return false;
}

int RPEngine::CollisionDetector::getCollisionNumber(
	RigidBody *bodyTarget, real deltaVelocity, 
	CollisionData *_data)
{
	if(!bodyTarget) return 0;
	int count = 0;
	for(int i = 0; i < _data->contactCount; i++)
	{
		RigidBody *contactBody1 = _data->contactArray[i].body[0];
		RigidBody *contactBody2 = _data->contactArray[i].body[1];

		if(contactBody1 == bodyTarget ||
		   contactBody2 == bodyTarget)
		{
			if(IsEqual(deltaVelocity, 0))
			{
				count++;
				continue;
			}
			
			Vector3 contactVector = _data->contactArray[i].contactNormal;
			real closingVelocity = GET_CLOSING_VELOCITY(
				contactVector, contactBody1, contactBody2);
			// colliding velocity
			if(deltaVelocity < closingVelocity)
				count++;
		}
	}
	return count;
}

RigidBody * RPEngine::CollisionDetector::getBodyCollisionWith(
	RigidBody *bodyTarget, real deltaVelocity,
	CollisionData *_data)
{

	if(!bodyTarget) return NULL;
	for(int i = 0; i < _data->contactCount; i++)
	{
		RigidBody *contactBody1 = _data->contactArray[i].body[0];
		RigidBody *contactBody2 = _data->contactArray[i].body[1];

		if(contactBody1 == bodyTarget ||
			contactBody2 == bodyTarget)
		{
			if(IsEqual(deltaVelocity, 0))
			{
				if(contactBody1 == bodyTarget)
					return _data->contactArray[i].body[1];
				if(contactBody2 == bodyTarget)
					return _data->contactArray[i].body[0];
			}

			Vector3 contactVector = _data->contactArray[i].contactNormal;
			real closingVelocity = GET_CLOSING_VELOCITY(
				contactVector, contactBody1, contactBody2);
			// colliding velocity
			if(deltaVelocity < closingVelocity)
			{
				if(contactBody1 == bodyTarget)
					return _data->contactArray[i].body[1];
				if(contactBody2 == bodyTarget)
					return _data->contactArray[i].body[0];
			}
		}
	}
	return NULL;
}

void RPEngine::CollisionDetector::getBodyArrayCollisionWith(
	RigidBody *bodyTarget, real deltaVelocity, 
	int limit, RigidBody *pBodyArray[], int *bodySize, 
	CollisionData *_data)
{
	if(limit == 0) return;
	if(!bodyTarget) return;
	
	int count = 0;
	for(int i = 0; i < _data->contactCount; i++)
	{
		RigidBody *contactBody1 = _data->contactArray[i].body[0];
		RigidBody *contactBody2 = _data->contactArray[i].body[1];
		if(IsEqual(deltaVelocity, 0))
		{
			if(contactBody1 == bodyTarget)
				pBodyArray[count++] = contactBody1;
			else if(contactBody2 == bodyTarget)
				pBodyArray[count++] = contactBody2;
			continue;
		}

		Vector3 contactVector = _data->contactArray[i].contactNormal;
			real closingVelocity = GET_CLOSING_VELOCITY(
				contactVector, contactBody1, contactBody2);

		if(deltaVelocity < closingVelocity)
		{
			if(contactBody1 == bodyTarget)
				pBodyArray[count++] = contactBody1;
			else if(contactBody2 == bodyTarget)
				pBodyArray[count++] = contactBody2;
		}

		if(limit == count) break;
	}
	*bodySize = count;
	return;
}

Contact * RPEngine::CollisionDetector::getCollisionContactData(
	RigidBody *bodyTarget, real deltaVelocity, 
	CollisionData *_data, int _size)
{
	if(!bodyTarget) return NULL;
	for(int i = 0; i < _data->contactCount; i++)
	{
		RigidBody *contactBody1 = _data->contactArray[i].body[0];
		RigidBody *contactBody2 = _data->contactArray[i].body[1];
		if(IsEqual(deltaVelocity, 0))
		{
			if(contactBody1 == bodyTarget)
				return &_data->contactArray[i];
			if(contactBody2 == bodyTarget)
				return &_data->contactArray[i];
		}

		Vector3 contactVector = _data->contactArray[i].contactNormal;
		real closingVelocity = GET_CLOSING_VELOCITY(
			contactVector, contactBody1, contactBody2);

		if(deltaVelocity < closingVelocity)
		{
			if(contactBody1 == bodyTarget)
				return &_data->contactArray[i];
			if(contactBody2 == bodyTarget)
				return &_data->contactArray[i];
		}
	}
	return NULL;
}

#undef CHECK_OVERLAP
#undef GET_CLOSING_VELOCITY
#endif
