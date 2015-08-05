#ifndef _PCONTACT_
#define _PCONTACT_

#include"pcontacts.h"

using namespace RPEngine;

/* contact */
void ParticleContact::resolve(real duration)
{
	resolveVelocity(duration);
	resolveInterPenetration(duration);
}

real ParticleContact::getRelativeVelocity() const
{
	Vector3 relativeVelocity = particle[0]->getVelocity();
	if(particle[1]) relativeVelocity -= particle[1]->getVelocity();
	return relativeVelocity * contactNormal; //inner product
}

void ParticleContact::resolveVelocity(real duration) // C.1.
{
	real beforeSepVelocity = getRelativeVelocity();
	if(beforeSepVelocity > 0) return; // either separating or stationary

	real afterSepVelocity = - beforeSepVelocity * restitution; //after impulse velocity

	// C.2. stationary handle
	Vector3 accCausedVelocity = particle[0]->getAcceleration();
	if(particle[1]) accCausedVelocity -= particle[1]->getAcceleration();
	real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

	if(accCausedSepVelocity < 0)
	{
		afterSepVelocity += restitution * accCausedSepVelocity;
		if(afterSepVelocity < 0) afterSepVelocity = 0;
	}


	real deltaVelocity = afterSepVelocity - beforeSepVelocity;// >=0 but not qual because of 'real' type
	//변화된속력=-결과속력(충돌계수-1)
	//변화된속력= 결과속력(1-충돌계수)로 테스트 해볼 것
	real totalInverseMass = particle[0]->getInverseMass();
	if(particle[1]) totalInverseMass += particle[1]->getInverseMass();
	if(totalInverseMass <= 0) return;

	//apply impulse
	real impulse = deltaVelocity / totalInverseMass; // g = m dot{p}, p=mv at this moment, 운동량의 변화량이나 여기서는 imass이므로 약간 다름

	Vector3 impulsePerIMass = contactNormal * impulse;

	//particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass();
	particle[0]->setVelocity(impulsePerIMass * particle[0]->getInverseMass() +
		particle[0]->getVelocity());

	if(particle[1])
		particle[1]->setVelocity(impulsePerIMass * (-particle[1]->getInverseMass()) +
		particle[1]->getVelocity());
}

void ParticleContact::resolveInterPenetration(real duration)
{
	if(penetration <= 0) return;

	real totalInverseMass = particle[0]->getInverseMass();
	if(particle[1]) totalInverseMass += particle[1]->getInverseMass();
	
	if(totalInverseMass <= 0) return; // infinity mass

	Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

	particle[0]->setPosition( particle[0]->getPosition() + 
							movePerIMass * particle[0]->getInverseMass() );
	if(particle[1]) //저자의 실수인 듯. -를 추가하였다.
		particle[1]->setPosition( particle[1]->getPosition() + 
								movePerIMass * (-particle[1]->getInverseMass()) );
}

/* contact resolver */
void ParticleContactResolver::setIterations(unsigned iterations)
{
	this->iterations = iterations;
}
void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, 
												unsigned int numContacts, 
												real duration)
{
	iterationsUsed = 0;
	while(iterationsUsed < iterations)
	{
		real maxSepVel = REAL_MAX;
		unsigned int maxIndex = numContacts;
		for(unsigned int i = 0; i < numContacts; i++)
		{
			real sepVel = contactArray[i].getRelativeVelocity();
			if(sepVel < maxSepVel &&
				(sepVel < 0 || contactArray[i].penetration > 0) )
			{
				maxSepVel = sepVel;
				maxIndex = i;
			}
		}
		if(maxIndex == numContacts) break;

		contactArray[maxIndex].resolve(duration);
		iterationsUsed++;
	}
}

#endif


/*
C.1.충격량-운동량 정리, impulse-momentum theorem

물체의 충격량 = 물체의 운동량의 변화량

따라서 반발계수(restitution)의 값에 의해 충돌 이후 물체의 운동량이 변화하게 된다. 
물체의 (운동량의 변화량) = (충돌전 운동량-충돌후 운동량) 을 통해 계산할 수 있고
그 결과값이 곧 impulse가 된다. 또한 작용-반작용의 법칙에 따라 두 물체가 받는 충격량은 동일하게 적용된다.
 
J = F dt = m a dt = m dv

하지만 소스에서는 (상대속도 변화량) / (두 물체의 질량의 역순의 합)을 통해 구현하고 있다.
(이 부분은 대학물리학 앞부분의 질량중심좌표계에서 본 충격량 공식을 참고바람)

참고 : http://terms.naver.com/entry.nhn?docId=1261226&cid=40942&categoryId=32229


C.2.접촉유지 해결

만약 직전 현재의 속도가 순수하게 직전 가속도에 의한 영향을 받은 결과인지를 판별한다.
만약 그렇다면 이 물체는 접촉을 유지하는 경우이며 그러므로 이후 상대 속도를 0으로 세팅시켜준다.

*/
