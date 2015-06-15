#ifndef _RIGIDBODY_
#define _RIGIDBODY_

#include"rbody.h"
#include<stdio.h>

#define GET_OBJECTS_FORCE
using namespace RPEngine;

void RigidBody::calculateDerivedData()
{
	orientation.normalize();
	_calculateTransformMatrix(transformMatrix, position, orientation);
	_transformInertiaTensor(inverseInertiaTensorWorld, orientation,
		inverseInertiaTensor, transformMatrix);
}

void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor)
{
	inverseInertiaTensor.setInverse(inertiaTensor);
}

void RigidBody::addForce(const Vector3 &force)
{
	forceAccum += force;
	isAwake = true;
}

//point -> wcs
void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 pt = point;
	pt -= position;

	forceAccum += force;
	torqueAccum += pt % force;
	isAwake = true;
}

//point -> mcs
void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point)
{
	Vector3 pt = getPointInWorldSpace(point);
	addForceAtPoint(force, pt);
	isAwake = true;
}

void RigidBody::integrate(real duration)
{
	// linear accel
	lastFrameAcceleration = acceleration;
	lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);
	// angular accel
	Vector3 angularAcceleration = inverseInertiaTensorWorld.transform(torqueAccum);

	// linear velocity
	velocity.addScaledVector(lastFrameAcceleration, duration);
	// angular velocity
	rotation.addScaledVector(angularAcceleration, duration);

	// damping only if the velocity had a value.
	if(!IsEqual(velocity.magnitude(), 0.f) || 
		!IsEqual(rotation.magnitude(), 0.f))
	{
		velocity *= real_pow(linearDamping, duration);
		rotation *= real_pow(angularDamping, duration);
	}
	
	// linear position
	position.addScaledVector(velocity, duration);
	// angular position
	orientation.addScaledVector(rotation, duration);

#ifdef GET_OBJECTS_FORCE 
	linearForce = forceAccum;
	angularForce = torqueAccum;
#endif
	
	calculateDerivedData();
	clearAccumulators();
}

void RigidBody::clearAccumulators()
{
	forceAccum.clear();
	torqueAccum.clear();
}

/* utility */
// these are all using angle(not radian) interface

Vector3 RigidBody::getPointInLocalSpace(const Vector3 &point) const
{
	return transformMatrix.transformInverse(point);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const
{
	return transformMatrix.transform(point);
}

void RigidBody::rotatePositionByAngleAxis(real angle, Vector3 axis) //World Axis
{
	angle = 3.141592f*angle/180.f;

	axis.normalize();

	Matrix3 rotMat(real_cos(angle) + axis.x*axis.x*(1-real_cos(angle)),
		axis.x*axis.y*(1-real_cos(angle)) - axis.z*real_sin(angle),
		axis.x*axis.z*(1-real_cos(angle)) + axis.y*real_sin(angle),

		axis.y*axis.x*(1-real_cos(angle)) + axis.z*real_sin(angle),
		real_cos(angle) + axis.y*axis.y*(1-real_cos(angle)),
		axis.y*axis.z*(1-real_cos(angle)) - axis.x*real_sin(angle),

		axis.z*axis.x*(1-real_cos(angle)) - axis.y*real_sin(angle),
		axis.z*axis.y*(1-real_cos(angle)) + axis.x*real_sin(angle),
		real_cos(angle) + axis.z*axis.z*(1-real_cos(angle))
		);
	position = (rotMat) * position;
}

void RigidBody::rotatePositionByAngleAxis(real angle, real x, real y, real z)
{
	angle = 3.141592f*angle/180.f;
	Vector3 axis(x,y,z);
	axis.normalize();

	Matrix3 rotMat(real_cos(angle) + axis.x*axis.x*(1-real_cos(angle)),
		axis.x*axis.y*(1-real_cos(angle)) - axis.z*real_sin(angle),
		axis.x*axis.z*(1-real_cos(angle)) + axis.y*real_sin(angle),

		axis.y*axis.x*(1-real_cos(angle)) + axis.z*real_sin(angle),
		real_cos(angle) + axis.y*axis.y*(1-real_cos(angle)),
		axis.y*axis.z*(1-real_cos(angle)) - axis.x*real_sin(angle),

		axis.z*axis.x*(1-real_cos(angle)) - axis.y*real_sin(angle),
		axis.z*axis.y*(1-real_cos(angle)) + axis.x*real_sin(angle),
		real_cos(angle) + axis.z*axis.z*(1-real_cos(angle))
		);
	position = (rotMat) * position;
}

void RigidBody::rotateOrientationByAngleAxis(real angle, Vector3 axis)
{
	orientation.rotateByAngleAxis(angle, axis);
}

void RigidBody::rotateOrientationByAngleAxis(real angle, real x, real y, real z)
{
	Vector3 axis(x, y, z);
	orientation.rotateByAngleAxis(angle, axis);
}

void RigidBody::rotateByAngleAxis(real angle, Vector3 axis)
{
	rotatePositionByAngleAxis(angle, axis);
	orientation.rotateByAngleAxis(angle, axis);
}

void RigidBody::rotateByAngleAxis(real angle, real x, real y, real z) // rotate position & orientation by the axis
{
	Vector3 axis(x,y,z);
	rotatePositionByAngleAxis(angle, axis); //position rotate around this axis
	orientation.rotateByAngleAxis(angle, axis);
}

/* get/set */

//position
void RigidBody::setPosition(Vector3 &position)
{
	this->position = position;
}

void RigidBody::setPosition(real x, real y, real z)
{
	this->position.x = x;
	this->position.y = y;
	this->position.z = z;
}

Vector3 &RigidBody::getPosition() //const
{
	return position;
}

void RigidBody::fillPosition(Vector3 *position) const
{
	*position = RigidBody::position;
}

void RigidBody::addScaledVectorPosition(Vector3 &direction, real scale)
{
	position.addScaledVector(direction, scale);
}

//velocity
void RigidBody::setVelocity(Vector3 &velocity)
{
	this->velocity = velocity;
}

void RigidBody::setVelocity(real x, real y, real z)
{
	this->velocity.x = x;
	this->velocity.y = y;
	this->velocity.z = z;
}

Vector3 &RigidBody::getVelocity()
{
	return this->velocity;
}

void RigidBody::fillVelocity(Vector3 *velocity) const
{
	*velocity = this->velocity;
}

void RigidBody::addVelocity(Vector3 &deltaVelocity)
{
	velocity += deltaVelocity;
}

//accel
void RigidBody::setAcceleration(Vector3 &acceleration)
{
	this->acceleration = acceleration;
}

void RigidBody::setAcceleration(real x, real y, real z)
{
	this->acceleration.x = x;
	this->acceleration.y = y;
	this->acceleration.z = z;
}

Vector3 &RigidBody::getAcceleration()
{
	return this->acceleration;
}

//mass
void RigidBody::setInverseMass(real inverseMass)
{
	this->inverseMass = inverseMass;
}

void RigidBody::setMass(const real mass)
{
	assert(mass != 0);
	this->inverseMass = static_cast<real>(1.0f) / mass;
}

real RigidBody::getInverseMass() const
{
	return inverseMass;
}

real RigidBody::getMass() const
{
	if(IsEqual(inverseMass, 0.f))
		return REAL_MAX;
	else
		return ((real)1.0) / inverseMass;
}
bool RigidBody::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

//rotation
void RigidBody::setRotation(Vector3 rotation)
{
	this->rotation = rotation;
}

void RigidBody::setRotation(real x, real y, real z)
{
	this->rotation.x = x;
	this->rotation.y = y;
	this->rotation.z = z;
}

Vector3 &RigidBody::getRotation()
{
	return this->rotation;
}

void RigidBody::addRotation(Vector3 &deltaRotation)
{
	rotation += deltaRotation;
}

//orientation
void RigidBody::setOrientation(Quaternion orientation)
{
	this->orientation = orientation;
}

Quaternion RigidBody::getOrientation()
{
	return this->orientation;
}

void RigidBody::addScaledVectorOrientation(Vector3 &direction, real scale)
{
	orientation.addScaledVector(direction, scale);
}

void RigidBody::setOrientationByAngleAxis(real angle, real x, real y, real z)
{
	Vector3 axis(x, y, z);
	orientation.setByAngleAxis(angle, axis);
}

void RigidBody::setOrientationByAngleAxis(real angle, Vector3 axis)
{
	orientation.setByAngleAxis(angle, axis);
}


//transform
Matrix4 RigidBody::getTransform()
{
	return this->transformMatrix;
}

void RigidBody::setTransform(Matrix4 transform)
{
	this->transformMatrix = transform;
}

void RigidBody::fillInverseInertiaTensorWorld(Matrix3 *tensor) const
{
	*tensor = inverseInertiaTensorWorld;
}
#endif 
