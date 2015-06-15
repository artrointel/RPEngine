#ifndef _PWORLD_
#define _PWORLD_

#include "pworld.h"

namespace RPEngine
{

	ParticleWorld::ParticleWorld(unsigned int _maxContacts, unsigned int iterations)
		: resolver(iterations), maxContacts(_maxContacts)
	{
		contacts = new ParticleContact[maxContacts];
		calculateIterations = (iterations == 0);
	}

	ParticleWorld::~ParticleWorld()
	{
		delete[] contacts;
	}

	void ParticleWorld::startFrame()
	{
		Particles::iterator p = particles.begin();
		for(;p != particles.end(); p++)
			(*p)->clearAccumulator();
	}

	unsigned int ParticleWorld::generateContacts()
	{
		unsigned int limit = maxContacts;
		ParticleContact *nextContact = contacts;

		for(PContactGenerators::iterator g = contactGenerators.begin();
			g != contactGenerators.end(); g++)
		{
			unsigned int used = (*g)->addContact(nextContact, limit);
			limit -= used;
			nextContact += used;

			if(limit <= 0) break;
		}
		return maxContacts - limit; // the number of contact used
	}

	void ParticleWorld::integrate(real duration)
	{
		Particles::iterator p = particles.begin();
		for(;p != particles.end(); p++)
		{
			(*p)->integrate(duration);
		}
	}

	void ParticleWorld::runPhysics(real duration)
	{
		registry.updateForces(duration); //apply forces
		integrate(duration); // calculate pos, vel, acc all particles 

		unsigned int usedContacts = generateContacts();
		if(usedContacts)
		{
			if(calculateIterations) resolver.setIterations(usedContacts * 2);
			resolver.resolveContacts(contacts, usedContacts, duration);
		}
	}
}


#endif
