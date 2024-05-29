
#include "world.h"
#include "pworld.h"

World::World(unsigned maxContacts, unsigned iterations)
	:
	firstContactGen(NULL),
	resolver(iterations),
	maxContacts(maxContacts)
{
	contacts = new Contact[maxContacts];
	calculateIterations = (iterations == 0);
}

World::~World()
{
	delete[] contacts;
}

unsigned World::generateContacts()
{
	unsigned limit = maxContacts;
	Contact* nextContact = contacts;

	ContactGenRegistration* reg = firstContactGen;
	while (reg)
	{
		unsigned used = reg->gen->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		// We've run out of contacts to fill. This means we're missing
		// contacts.
		if (limit <= 0) break;

		reg = reg->next;
	}

	// Return the number of contacts used.
	return maxContacts - limit;
}

//void World::startFrame()
//{
//	for (RigidBodies::iterator b = bodies.begin(); b != bodies.end(); b++)
//	{
//		b->clearAccumulators();
//		b->calculateDerivedData();
//	}
//}

void World::startFrame()
{
	BodyRegistration* reg = firstBody;
	while (reg)
	{
		// Remove all forces from the accumulator
		reg->body->clearAccumulators();
		reg->body->calculateDerivedData();

		// Get the next registration
		reg = reg->next;
	}
}


void World::integrate(real duration)
{
	for (RigidBodies::iterator b = bodies.begin(); b != bodies.end(); b++)
	{
		//Integrate the body by the given duration.
		b->integrate(duration);
	}
}

void World::runPhysics(real duration)
{
	// First apply the force generators
	//registry.updateForces(duration);

	// Then integrate the objects
	BodyRegistration* reg = firstBody;
	while (reg)
	{
		// Remove all forces from the accumulator
		reg->body->integrate(duration);

		// Get the next registration
		reg = reg->next;
	}

	// Generate contacts
	unsigned usedContacts = generateContacts();

	// And process them
	if (calculateIterations) resolver.setIterations(usedContacts * 4);
	resolver.resolveContacts(contacts, usedContacts, duration);
}

//void World::runPhysics(real duration)
//{
//	//First apply the force generators.
//	registry.updateForces(duration);
//
//	//Then integrate the objects.
//	integrate(duration);
//}