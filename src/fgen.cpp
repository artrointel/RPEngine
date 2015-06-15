#ifndef _FGEN_
#define _FGEN_

#include "fgen.h"

using namespace RPEngine;

/* registry */
void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
	ForceGeneration registration;
	registration.body = body;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ForceRegistry::remove(RigidBody *body, ForceGenerator *fg)
{
	Registry::iterator iter = registrations.begin();
	for(; iter != registrations.end(); iter++)
	{
		if(iter->body == body &&
			iter->fg == fg)
		{
			registrations.erase(iter);
			break;
		}
	}
}

void ForceRegistry::clear()
{
	registrations.clear();
}

void ForceRegistry::updateForces(real duration)
{
	Registry::iterator iter = registrations.begin();
	for(;iter != registrations.end(); iter++)
	{
		iter->fg->updateForce(iter->body, duration);
	}
}

/* gravity */
void Gravity::updateForce(RigidBody *body, real duration)
{
	if(!body->hasFiniteMass()) return;
	body->addForce(gravity * body->getMass());
}
/* drag */
void SimpleDrag::updateForce(RigidBody *body, real duration)
{
	Vector3 force;
	body->fillVelocity(&force);

	//let drag coefficient defines 
	real dragCoeff = force.magnitude(); //scalar value of velocity

	dragCoeff = k1*dragCoeff + k2*dragCoeff*dragCoeff;
	force.normalize();

	force = force*dragCoeff;
	force *= -1;

	Vector3 angularVelocity = body->getRotation();
	body->setRotation(angularVelocity*0.97);
}
/* spring */
void Spring::updateForce(RigidBody *body, real duration)
{
	Vector3 lws = body->getPointInWorldSpace(connectionPoint);
	Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);
	Vector3 force = lws - ows;

	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength);
	magnitude *= springConstant;

	force.normalize();
	force *= -magnitude;
	body->addForceAtPoint(force, lws);
}

void RPEngine::HardSpring::updateForce(RigidBody *body, real duration)
{
	if(isDestroy == true) return;

	Vector3 lws = body->getPointInWorldSpace(connectionPoint);
	Vector3 ows = other->getPointInWorldSpace(otherConnectionPoint);
	Vector3 force = lws - ows;

	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength);
	
	if(magnitude > destroyLength) //destroy
	{
		isDestroy = true;
		magnitude *= springConstant;
		force.normalize();
		force *= magnitude/2.f;
		force += (Vector3(-10*magnitude/2.f,0,0));
		body->addForceAtPoint(force, lws);
		force *= -1;
		other->addForceAtPoint(force, ows);
		return;
	}
	
	magnitude = linearCoeff * magnitude + magnitude * magnitude * springConstant; //magni~
	
	force.normalize();
	force *= -magnitude/2.f;
	body->addForceAtPoint(force, lws);
	force *= -1;
	other->addForceAtPoint(force, ows);
}

void Aero::updateForce(RigidBody *body, real duration)
{
	Aero::updateForceFromTensor(body, duration, tensor);
}

void Aero::updateForceFromTensor(RigidBody *body, real duration, const Matrix3 &tensor)
{
	Vector3 velocity = body->getVelocity();
	velocity += *windspeed;

	//in mcs
	Vector3 bodyVelocity = body->getTransform().transformInverseDirection(velocity);
	Vector3 bodyForce = tensor.transform(bodyVelocity);
	Vector3 force = body->getTransform().transformDirection(bodyForce);

	body->addForceAtBodyPoint(force, position);
}

/* aero control */
Matrix3 AeroControl::getTensor()
{
	if(controlSetting <= -1.0f) return minTensor;
	else if(controlSetting >= 1.0f) return maxTensor;
	else if(controlSetting < 0)
	{
		return Matrix3::linearInterpolate(minTensor,
										  tensor,
										  controlSetting+1.0f);
	}
	else return tensor;
}

void AeroControl::updateForce(RigidBody *body, real duration)
{
	Matrix3 tensor = getTensor();
	Aero::updateForceFromTensor(body, duration, tensor);
}

/* CustomForce */
void CustomForce::updateForce(RigidBody *body, real duration)
{
	(this->*pUpdateForce)(body, duration);
}

#endif
