#include"pfgen.h"

using namespace RPEngine;

/* particle force registry */
void ParticleForceRegistry::updateForces(real duration)
{
	Registry::iterator i = registrations.begin();

	for(; i != registrations.end(); i++)
		i->fg->updateForce(i->particle, duration);
}

void ParticleForceRegistry::add(Particle *particle, ParticleForceGenerator *fg)
{
	ParticleForceGeneration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ParticleForceRegistry::remove(Particle *particle, ParticleForceGenerator *fg)
{
	Registry::iterator iter = registrations.begin();
	for(; iter != registrations.end(); iter++)
	{
		if(iter->particle == particle 
			&& iter->fg == fg)
		{
			registrations.erase(iter);
			break;
		}
	}
}

void ParticleForceRegistry::clear()
{
	registrations.clear(); 
}

/* gravity */
void ParticleGravity::updateForce(Particle *particle, real duration)
{
	if(!particle->hasFiniteMass()) return;

	particle->addForce(gravity * particle->getMass());
}

/*  rain */
void ParticleRainDrop::updateForce(Particle *particle, real duration)
{
	if(!particle->hasFiniteMass()) return;
	if(particle->getPosition().y < ground)
	{
		Vector3 pos(particle->getPosition());
		Vector3 velocity(particle->getVelocity());
		pos.y = cloud;
		particle->setPosition(pos);
		return;
	}

	//particle->addForce(gravity * particle->getMass());
}


/* drag */
void ParticleDrag::updateForce(Particle *particle, real duration)
{
	Vector3 force;
	particle->getVelocity(&force); //copy velocity

	real dragCoeff = force.magnitude();
	dragCoeff = k1*dragCoeff + k2*dragCoeff*dragCoeff;

	force.normalize();
	force *= -dragCoeff;

	particle->addForce(force);
}

/* spring */
void ParticleSpring::updateForce(Particle *particle, real duration)
{
	Vector3 force;
	particle->getPosition(&force);

	force -= other->getPosition(); // vector force be l_0

	real magnitude = force.magnitude();
	magnitude = real_abs(magnitude - restLength); // abs means only 'push' force
	magnitude *= springConstant;

	force.normalize();
	force *= -magnitude;
	particle->addForce(force);
}

/* anchored spring */
void ParticleAnchoredSpring::updateForce(Particle *particle, real duration)
{
	
	Vector3 force;
	particle->getPosition(&force);
	force -= *anchor; // might equal to the ParticleSpring::updateForce above

	real magnitude = force.magnitude();
	magnitude = (restLength - magnitude) * springConstant; //

	force.normalize();
	force *= magnitude;

	particle->addForce(force);
}

/* bungee */
void ParticleBungee::updateForce(Particle *particle, real duration)
{
	Vector3 force;
	particle->getPosition(&force);
	force -= other->getPosition();

	real magnitude = force.magnitude();
	if(magnitude <= restLength) return;

	magnitude = springConstant * (restLength - magnitude);

	force.normalize();
	force *= -magnitude;

	particle->addForce(force);
}

/* buoyancy */
void ParticleBuoyancy::updateForce(Particle *particle, real duration)
{
	real depth = particle->getPosition().y;
	if(depth >= waterHeight + maxDepth) return; // out of the water
	Vector3 force(0,0,0);

	if(depth <= waterHeight - maxDepth)
	{
		force.y = liquidDensity * volume;
		particle->addForce(force);
		return;
	}

	force.y = liquidDensity * volume *
		(depth - maxDepth - waterHeight) / 2 * maxDepth;
	particle->addForce(force);
	
}
