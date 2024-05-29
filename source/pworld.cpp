

#include "pworld.h"

unsigned ParticleWorld::generateContacts()
{
	unsigned limit = maxContacts;
	ParticleContact* nextContact = contacts;

	for (ContactGenerators::iterator g = contactGenerators.begin(); g != contactGenerators.end(); g++)
	{
		unsigned used = (*g)->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		//We hace run out of contacts to fill. This means weare missing
		//contacts.
		if (limit <= 0)
		{
			break;
		}
	}

	//Return the number of contacts used.
	return maxContacts - limit;
}

void ParticleWorld::integrate(real duration)
{
	for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
	{
		//integrate the particle by the given duration.
		(*p)->integrate(duration);
	}
}

void ParticleWorld::runPhysics(real duration)
{
	//First, apply the force generators.
	registry.updateForces(duration);

	//Then integrate the objects.
	integrate(duration);

	//Generate contacts.
	unsigned usedContacts = generateContacts();

	//And process them.
	if (usedContacts)
	{
		if (calculateIterations)
		{
			resolver.setIterations(usedContacts * 2);
		}

		resolver.resolveContacts(contacts, usedContacts, duration);
	}

}