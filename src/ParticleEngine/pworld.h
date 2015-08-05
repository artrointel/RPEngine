#ifndef _PWORLD_H
#define _PWORLD_H

#include <vector>
#include "particle.h"
#include "pcontacts.h"
#include "pfgen.h"

namespace RPEngine
{
	class ParticleWorld
	{
	public:
		typedef std::vector<Particle *> Particles;
		typedef std::vector<ParticleContactGenerator *> PContactGenerators;

	public:
		Particles particles;
		PContactGenerators contactGenerators;
		
		ParticleForceRegistry registry;
		ParticleContactResolver resolver;
		bool calculateIterations; // true if resolver should calculate at that frame

		ParticleContact *contacts;
		unsigned int maxContacts;

	public:
		explicit ParticleWorld(unsigned int _maxContacts, unsigned int iterations = 0);
		virtual ~ParticleWorld();
	public:
		void startFrame(); // clear all particle's forces
		unsigned int generateContacts();
		void integrate(real duration);
		void runPhysics(real duration);
	};
}

#endif
