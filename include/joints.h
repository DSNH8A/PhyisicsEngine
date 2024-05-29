#pragma once


#include "core.h"
#include "contacts.h"


/*
* Joints link together two rigid bodies and ensure they do not
* separate. In physics engine there may be many
* different types of joints that reduce the number of relative
* degrees of freedom between two objects. This joint is a common
* position joint--each object has a location (given in 
* body coordinates) that will be kept at hte same point in the 
* simulation.
*/
class Joint : public ContactGenerator
{
public:
	/*
	* Holds the two rigid bodies that are connected by this joint.
	*/
	RigidBody* body[2];

	/*
	* Holds the relative location of the connnection for each
	* body, given in local coordinates.
	*/
	Vector3 position[2];

	/*
	* Holds the the maximum displacement at the joint before the
	* joint is considered to be violated. This is normally a 
	* small epsilon value. It can be larger, however in which 
	* case the joint will behave as if an inelastic cables joined
	* the bodies at their joint locations.
	*/
	real error;

	/*
	* configures the joint in one go.
	*/
	void set(RigidBody* a, const Vector3& a_pos, RigidBody* b, const Vector3& b_pos, real error);

	/*
	* Generates the contacts required to restore the joint if it
	* has been violated.
	*/
	unsigned addContact(Contact* contact, unsigned limit) const;


};