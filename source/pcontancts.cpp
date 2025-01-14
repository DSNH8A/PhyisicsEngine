
#include "pcontancts.h"
#include "precision.h"

void ParticleContact::resolve(real duration)
{
	resolveVelocity(duration);
	resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
	Vector3 relaitveVelocity = particle[0]->getVelocity();

	if (particle[1])
	{
		relaitveVelocity -= particle[1]->getVelocity();
	}

	return relaitveVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration)
{
	//Find the velocity in the direction of the contact.
	real separatingVelocity = calculateSeparatingVelocity();

	//Check if it needs to be resolved.
	if (separatingVelocity > 0)
	{
		//The contact is either separating, or stationary
		//no impulse is required.
		return;
	}

	//Calculate the new separating velocity.
	real newSepVelocity = -separatingVelocity * restitution;

	real deltaVelocity = newSepVelocity - separatingVelocity;

	//We apply the change in velocity to each object in proportion to
	//theit inverse mass(i.e. those with lower inverse mass [hihger
	//actual mass] get less change in velocity).
	real totalInverseMass = particle[0]->getInverseMass();

	if (particle[1])
	{
		totalInverseMass += particle[1]->getInverseMass();
	}

	//If all particles have infinite mass, then impulses have no effect.
	if (totalInverseMass <= 0)
	{
		return;
	}

	//Calculate the impulse to apply.
	real impulse = deltaVelocity / totalInverseMass;

	//Find the amount of impulse per unit of inverse mass.
	Vector3 impulsePerIMass = contactNormal * impulse;

	//Apply impulses: they are applied in the direction of the contact, 
	//and are proportional to the inverse mass.
	particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());

	if (particle[1])
	{
		//Particle goes in the opposite direction
		particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass());
	}
}

void ParticleContact::resolveInterpenetration(real duration)
{
	//If we dont have any interpenetration, skip this step.
	if (penetration <= 0)
	{
		return;
	}

	//The movement of each object is based on their inverse mass,
	//so total that.
	real totalInverseMass = particle[0]->getInverseMass();

	if (particle[1])
	{
		totalInverseMass += particle[1]->getInverseMass();
	}

	//If all particles have infinite mass, then we do nothing.
	if (totalInverseMass <= 0)
	{
		return;
	}

	//Find the amount of penetration resolution per unit
	//of inverse mass.
	Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

	//Calculate the movement amounts.
	particleMovement[0] = movePerIMass * particle[0]->getInverseMass();

	if (particle[1])
	{
		particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
	}

	else
	{
		particleMovement[1].clear();
	}

	//Apply the penetration resolution.
	particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);

	if (particle[1])
	{
		particle[1]->getPosition(particle[1]->getPosition() + particleMovement[1]);
	}
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
	:
	iterations(iterations)
{
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
	ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact* contactArray, unsigned numContacts, real duration)
{
	unsigned i;

	iterationsUsed = 0;

	while (iterationsUsed < iterations)
	{
		//Find the contact with the largest closing velocity.
		real max = REAL_MAX;
		unsigned maxIndex = numContacts;

		for (i = 0; i < numContacts; i++)
		{
			real sepVel = contactArray[i].calculateSeparatingVelocity();

			if (sepVel < max && (sepVel < 0 || contactArray[i].penetration >	0))
			{
				max = sepVel;
				maxIndex = i;
			}
		}

		//Do we have anything worth resolveing?
		if (maxIndex == numContacts)
		{
			break;
		}

		//Resolve this contact.
		contactArray[i].resolve(duration);

		iterationsUsed++;
	}
}