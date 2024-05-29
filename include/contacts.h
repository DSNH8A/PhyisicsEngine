#pragma once

#include "core.h"

/*
* A contat represents two bodies in contact. Resolving a 
* contact removes their interpeneration, and applies sufficient
* inpulse to keep them apart. Colliding bodies amy also rebound.
* Contacts can be used to represent positional joints, by making
* the contact constraint keep the bodies in their correct orientation.
*/
class Contact
{
public:
	/*
	* Holds the position of the contact in wordl coordinates.
	*/
	Vector3 contactPosition;

	/*
	* Holds the direction of the contact in world doordinates.
	*/
	Vector3 contactNormal;

	/*
	* Holds the depth of penetration at the contact point. If both
	* bodies are specified, then contact point should be midway
	* between the interpenetrating points.
	*/
	real penetration;

	/*
	* Holds the lateral friction coefficient at the contact.
	*/
	real friction;

	/*
	* Holds the position of the contact in world coordinates.
	*/
	Vector3 contactPoint;

	/*
	* Holds the normal restitution coefficient at the contact.
	*/
	real restitution;

	/*
	* Holds the bodies that are involved in the contact.
	* The scond of theese can be null for contacts with scenery.
	*/
	RigidBody* body[2];

	/*
	* Holds the world space position of the contact point relative to
	* center of each body. This is set when the calculateInternals
	* function is run.
	*/
	Vector3 relativeContactPosition[2];

	/*
	* A transform matrix that converts co-ordinates in the contact's
	* frame of reference to world co-ordinates. The columns of this
	* matrix form an orthonormal set of vectors.
	*/
	Matrix3 contactToWorld;

	/*
	* Holds the closing velocity at the point of contact. This is set
	* when the calculateInternals function is run.
	*/
	Vector3 contactVelocity;

	/*
	* Holds the required change in velocity for this contact to be
	* resolved.
	*/
	real desiredDeltaVelocity;

	/*
	* The contact resolver objects needs acess into the contacts to 
	* set and affect the contacts.
	*/
	/*friend ContactResolver;*/

public:

	/*
	* Calculates internal data from state data. This is called before
	* the resolution algorithm tries to do any resolution. It should never
	* need to be called manually.
	*/
	void calculateInternals(real duration);

	/*
	* Reverses the contact. This invoolves swapping the two rigid
	* bodies and reversing the contact normal. The internal 
	* values should the be recalculated using calculateinternals
	* (this is not done automatically, as this method may be
	* called from calcualteInternals).
	*/
	void swapBodies();

	/*
	* Calculates and returns the velocity of the contact
	* point on the given body.
	*/
	Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);

	/*
	* Resolves the positional issues with the given array of contraints,
	* using the given number of iterations. 
	*/
	void adjustPositions(Contact* c, unsigned numContacts, real duration);

	/*
	* Sets the data that doesn't normalyy depend on the position
	* of the contact (i.e. the bodies, and their material properties).
	*/
	void setBodyData(RigidBody* one, RigidBody* two,
		real friction, real restitution);

	/*
	* Calculates an orthonormal basis fot hte the contact point, based on
	* the primary friction direction (for anisotropic friction) or a 
	* random orientation (for isotropic friction).
	*/
	void calculateContactBasis();

	
	/*
	* Calculates the impulse needed to resolve this contact,
	* given that the contact has no friction. A pair of inertia
	* tensors - one for each contact object - is specified to
	* save calculation time: the calling function has access to
	* these anyway.
	*/
	Vector3 calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor);

	/*
	* Performs an inertia weighted penetration resolution of this
	* contact alone.
	*/
	void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);

	/*
	* Applies an impulse to the given body, returning the
	* change in velocities.
	*/
	void applyImpulse(const Vector3& impulse, RigidBody* body, Vector3* velocityChange, Vector3* rotationChange);

	/*
	* Performs an inertia-weighted impulse based resolution of this
	* contact alone.
	*/
	void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);

	/*
	* Calculates and sets the internal value for the desired delta
	* velocity. Stops resting objects from vibrating on the ground
	* or on other resting objetcs.
    */
	void calculateDesiredDeltaVelocity(real duration);

	/*
	* Calculates the impulse needed to resolve this contact,
	* given that the contact has a non-zero coefficient of
	* friction. A pair of inertia tensors-one for each contact
	* object-is specified to save calculation time. The calling 
	* function has access to these anyway.
	*/
	Vector3 calculateFrictionImpulse(Matrix3* inverseInertiaTensor);

	/*
	* Updates the awake state of rigid bodies that are taking
	* place in the given contact. A body will be made awake if it
	* is in contact with a body that is awake.
	*/
	void matchAwakeState();
};


 /*
 * The contact resolution. One resolver intance
 * can be shared for the entire simulataion, as long
 * as you need roughly the same parameters each time
 * which is normal.
 */
class ContactResolver
{
public:

	/*
	* Holds the number of iterations to perform when resolving
	* velocity.
	*/
	unsigned velocityIterations;

	/*
	* Holds the number of iterations to perform when resolving
	* position.
	*/
	unsigned positionIterations;

	/*
	* To avoid instability velocities smaller
	* than this value are considered to be zero. Too small and the
	* simulation may be unstable, too large and the bodies may
	* interpenetrate visually. A good starting point is the default
	* of 0.01.
	*/
	real velocityEpsilon;

	/*
	* To avoid instability penetrations
	* smaller than this value are considered to be not interpenetrating.
	* Too small and the simulation may be unstable, too large and the
	* bodies may interpenetrate visually. A good starting point is
	* the default of0.01.
	*/
	real positionEpsilon;

	/*
	* Stores the number of velocity iterations used in the
	* last call to resolve contacts.
	*/
	unsigned velocityIterationsUsed;

	/*
	* Stores the number of position iterations used in the
	* last call to resolve contacts.
	*/
	unsigned positionIterationsUsed;

private:
	/**
	 * Keeps track of whether the internal settings are valid.
	 */
	bool validSettings;

public:

	/*
	* Creates a new contact resolver with the given number of iterations
    * per resolution call, and optional epsilon values.
    */
	ContactResolver(unsigned iterations,real velocityEpsilon = (real)0.01,real positionEpsilon = (real)0.01);

	/*
	* Creates a new contact resolver with the given number of iterations
	* for each kind of resolution, and optional epsilon values.
	*/
	ContactResolver(unsigned velocityIterations,unsigned positionIterations,real velocityEpsilon = (real)0.01,real positionEpsilon = (real)0.01);


	/*
	* Returns true if the resolver has valid settings and is ready to go.
    */
	bool isValid()
	{
		return (velocityIterations > 0) &&
			(positionIterations > 0) &&
			(positionEpsilon >= 0.0f) &&
			(positionEpsilon >= 0.0f);
	}

	/*
	* Sets the number of iterations for each resolution stage.
	*/
	void setIterations(unsigned velocityIterations,
		unsigned positionIterations);

	/*
	* Sets the number of iterations for both resolution stages.
	*/
	void setIterations(unsigned iterations);

	/*
	* Sets the tolerance value for both velocity and position.
	*/
	void setEpsilon(real velocityEpsilon,
		real positionEpsilon);

	/*
	* Resolves a st of contacts for bvoth penetration and
	* velocity. contacts that cannot interact with each other
	* should be passed to separate calls to resolveContacts, as
	* the resolution algorithm takes much longer for lots of 
	* contacts than it does for the same number of contacts in
	* small sets.
	*/
	void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);

	/*
	* Sets up contacts ready for processing. This ensures that their
	* internal data is configured correctly and the correct set of
	* bodies is made alive.
	*/
	void prepareContacts(Contact* contactArray, unsigned numContacts, real duration);

	/*
	* Resolves the positional issues with the given array of constraints,
	* using the given number of iterations.
	*/
	void adjustPositions(Contact* contacts, unsigned numContacts, real duration);

	/*
	* Resolves the velocity issues with the given array of constraints,
	* using the given number of iterations.
	*/
	void adjustVelocities(Contact* contactArray, unsigned numContacts, real duration);
};

class ContactGenerator
{
public:
	/**
	 * Fills the given contact structure with the generated
	 * contact. The contact pointer should point to the first
	 * available contact in a contact array, where limit is the
	 * maximum number of contacts in the array that can be written
	 * to. The method returns the number of contacts that have
	 * been written.
	 */
	virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
};