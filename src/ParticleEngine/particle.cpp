
#include "assert.h"
#include "particle.h"
using namespace RPEngine;

void Particle::integrate(real duration) // C.1.
{
	if(inverseMass <= 0.0f) return;
	assert(duration > 0.0);

	position.addScaledVector(velocity, duration); // 1. position by velocity

	Vector3 resultingAcc = acceleration;
	resultingAcc.addScaledVector(forceAccum, inverseMass); // 2. resulting accumulate by external force 
	
	velocity.addScaledVector(resultingAcc, duration); // 3. velocity by acceleration
	velocity *= real_pow(damping, duration);

	clearAccumulator();
}

void Particle::addForce(Vector3 &force)
{
	forceAccum += force;
}

void Particle::addForce(real x, real y, real z)
{
	forceAccum.x += x;
	forceAccum.y += y;
	forceAccum.z += z;
}

void Particle::clearAccumulator()
{
	forceAccum.clear();
}

/* simply get/set */
void Particle::setPosition(Vector3 &position)
{
	this->position = position;
}

void Particle::setPosition(real x, real y, real z)
{
	this->position.x = x;
	this->position.y = y;
	this->position.z = z;
}

Vector3 &Particle::getPosition() //const
{
	return position;
}

void Particle::getPosition(Vector3 *position) const
{
	*position = Particle::position;
}

void Particle::setVelocity(Vector3 &velocity)
{
	this->velocity = velocity;
}

void Particle::setVelocity(real x, real y, real z)
{
	this->velocity.x = x;
	this->velocity.y = y;
	this->velocity.z = z;
}

Vector3 &Particle::getVelocity() //const
{
	return velocity;
}

//구태여 이렇게 사용할 필요가 있을 지
void Particle::getVelocity(Vector3 *velocity) const
{
	*velocity = Particle::velocity;
}

void Particle::setAcceleration(Vector3 &acceleration)
{
	this->acceleration = acceleration;
}

void Particle::setAcceleration(real x, real y, real z)
{
	this->acceleration.x = x;
	this->acceleration.y = y;
	this->acceleration.z = z;
}

const Vector3 &Particle::getAcceleration()
{
	return acceleration;
}

void Particle::setInverseMass(real inverseMass)
{
	this->inverseMass = inverseMass;
}

void Particle::setMass(const real mass)
{
	assert(mass != 0);
	this->inverseMass = static_cast<real>(1.0) / mass;
}

real Particle::getInverseMass() const
{
	return inverseMass;
}

real Particle::getMass() const
{
	if(inverseMass == 0)
		return REAL_MAX;
	else
		return ((real)1.0) / inverseMass;
}

//check this func
bool Particle::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

void Particle::setDamping(real damping)
{
	this->damping = damping;
}
real Particle::getDamping() const
{
	return damping;
}

/*
C.1. 물체의 위치와 속도를 계산하는 알고리즘에서
위치를 먼저 계산하고 속도를 계산하는 이유는 물체의 위치를 결정짓는 순간 외력이 존재했다면, 
그 외력에 의해 물체의 위치는 변하지 않는다는 정의를 보존하기 위함이다.

*/
