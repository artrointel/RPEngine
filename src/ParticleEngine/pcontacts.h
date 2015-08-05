#ifndef _PCONTACT_H
#define _PCONTACT_H

#include"particle.h"
#include"precision.h"

namespace RPEngine
{

	class ParticleContact // particle[0] perspective
	{
	public:
		Particle *particle[2]; // grap the particles that are in the contact, another one can be NULL if that particle was like a wall.
		real restitution; // 'particle A's normal restitution coefficient
		real penetration; // depth of penetration at the contact
		Vector3 contactNormal; // direction of resulting this collision
		

	public:
		void resolve(real duration); // contact -> interpenetration resolve
		real getRelativeVelocity() const; // < 0, contact before

	private:
		void resolveVelocity(real duration); //collision resolve
		void resolveInterPenetration(real duration); //penetration resolve and not using duration actually
	};


	class ParticleContactResolver
	{
	protected:
		unsigned int iterations; // the number of iteration allowed
		unsigned int iterationsUsed; // current iter used
	public:
		explicit ParticleContactResolver(unsigned int _iterations) : iterations(_iterations) {}
		
		void setIterations(unsigned iterations);
		void resolveContacts(ParticleContact *contactArray, unsigned int numContacts, real duration);

	};


	class ParticleContactGenerator
	{
	public:
		virtual unsigned int addContact(ParticleContact *contact,
										unsigned int limit) const = 0;
	};
}

#endif	
