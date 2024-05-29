#pragma once

#include "particle.h"

/*
* A contact represents two objects in contatct (in this case
* particleContact representing two particles).Resolving a 
* contact remove their interpenetration, and applies sufficient
* impulse to keep them apart. Colliding bodies may also rebound.
* 
* The contact has no callable functions, it just holds the
* contact details. To resolve a set of contacts, use the particle
* contact resolver class.
*/
class ParticleContact
{
public:
	/*
	* Holds the particles that are involved in the contact. The
	* second of these can be NULL for contacts with the scenery.
	*/
	Particle* particle[2];

	Vector3 particleMovement[2];

	/*
	* Holds the normal restitution coefficient at the contact.
	*/
	real restitution;

	/*
	* Holds the direction of the contact in world coordinates.
	*/
	Vector3 contactNormal;

public:

	/*
	* Resolves this contact for both velocity and interpenetration.
	*/
	void resolve(real duration);

	/*
	* Calculate the separating velocity at this contact.
	*/
	real calculateSeparatingVelocity() const;

	/*
	* Holds the depth of penetration at the contact.
	*/
	real penetration;

	/*
	* Handles the impulse calculations for this collisions.
	*/
	void resolveVelocity(real duration);

	/*
	* Handles the interpenetration resolution for this contact.
	*/
	void resolveInterpenetration(real duration);
};

/*
* the contact resolution routine for particle contacts. One
* resolver instance can be shared for the entire simulation.
*/
class ParticleContactResolver
{
public:
	/*
	* Holds the number of iterations allowed.
	*/
	unsigned iterations;

	/*
	* This is a performance tracking value; we kepp record
	* of the actual number of iterations used.
	*/
	unsigned iterationsUsed;

public:

	/*
	* Creates a new contact resolver
	*/
	ParticleContactResolver(unsigned iterations);

	/*
	* Sets the number of iterations that can be used. 
	*/
	void setIterations(unsigned iterations);

	/*
	* Resolves a set of contacts for both penetration 
	* and velocity.
	*/
	void resolveContacts(ParticleContact* contactArray, unsigned numContects, real duration);
};

class ParticleContactGenerator
{

public:

	/*
	* Fills the given contact structure with the gerated
	* contact. The contact pointer should point to the first
	* available contact int the contatc array, where limit is the
	* maximum number of contacts in the array that can be writeen
	* to. The method returns the number of contacts that have
	* benn written.
	*/
	virtual unsigned addContact(ParticleContact* contact, unsigned limit) const = 0;
};