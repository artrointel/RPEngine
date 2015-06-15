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
		real restitution; // C.1. 'particle A's normal restitution coefficient, "반발계수"
		real penetration; // depth of penetration at the contact
		Vector3 contactNormal; // direction of resulting this collision
		

	public:
		//생성자 및 소멸자 추가 예정
		void resolve(real duration); // contact -> interpenetration resolve
		real getRelativeVelocity() const; // < 0, contact before

	private:
		void resolveVelocity(real duration); //collision resolve
		void resolveInterPenetration(real duration); //penetration resolve and not using duration actually
	};


	class ParticleContactResolver // 추 후 Contact 상속/friend시킬 가능성O
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


/*
C.1. 반발계수 e , restitution coefficient - 물체의 재질에 의해 결정
http://terms.naver.com/entry.nhn?docId=1098679&cid=40942&categoryId=32227

e = - (v1`-v2`) / (v1-v2) for vector3 vx


- 반발계수와 무관하게 운동량 보존 법칙을 만족하나, 운동에너지는 다른 에너지로의 전환이 반발계수에 따라 달라진다.
- 음이면 운동의 방향이 변화되는 것을 의미
0이면 완전비탄성충돌
1이면 완전탄성충돌

*/
