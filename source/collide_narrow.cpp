

#include "collide_narrow.h"
#include "precision.h"

void CollisionPrimitive::calculateInternals()
{
	transform = body->getTransform() * offset;
}

bool IntersectionTests::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane)
{
	// Find the distance from the origin
	real ballDistance = plane.direction * sphere.getAxis(3) - sphere.radius;

	// Check for the intersection
	return ballDistance <= plane.offset;
}

bool IntersectionTests::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two)
{
	// Find the vector between the objects
	Vector3 midline = one.getAxis(3) - two.getAxis(3);

	// See if it is large enough.
	return midline.squaredMagnitude() < (one.radius + two.radius) * (one.radius + two.radius);
}

static inline real transformToAxis(const CollisionBox& box, const Vector3& axis)
{
	return
		box.halfSize.x * real_abs(axis * box.getAxis(0)) +
		box.halfSize.y * real_abs(axis * box.getAxis(1)) +
		box.halfSize.z * real_abs(axis * box.getAxis(2));
}

/**
 * This function checks if the two boxes overlap
 * along the given axis. The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline bool overlapOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCentre)
{
	// Project the half-size of one onto axis
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	// Project this onto the axis
	real distance = real_abs(toCentre * axis);

	// Check for overlap
	return (distance < oneProject + twoProject);
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(
	const CollisionBox& one,
	const CollisionBox& two
)
{
	// Find the vector between the two centres
	Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

	return (
		// Check on box one's axes first
		TEST_OVERLAP(one.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(2)) &&

		// And on two's
		TEST_OVERLAP(two.getAxis(0)) &&
		TEST_OVERLAP(two.getAxis(1)) &&
		TEST_OVERLAP(two.getAxis(2)) &&

		// Now on the cross products
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
		);
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane)
{
	// Work out the projected radius of the box onto the plane direction
	real projectedRadius = transformToAxis(box, plane.direction);

	// Work out how far the box is from the origin
	real boxDistance = plane.direction * box.getAxis(3) - projectedRadius;

	// Check for the intersection
	return boxDistance <= plane.offset;
}

unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Cache the sphere position
	Vector3 position = sphere.getAxis(3);

	// Find the distance from the plane
	real centreDistance = plane.direction * position - plane.offset;

	// Check if we're within radius
	if (centreDistance * centreDistance > sphere.radius * sphere.radius)
	{
		return 0;
	}

	// Check which side of the plane we're on
	Vector3 normal = plane.direction;
	real penetration = -centreDistance;
	if (centreDistance < 0)
	{
		normal *= -1;
		penetration = -penetration;
	}
	penetration += sphere.radius;

	// Create the contact - it has a normal in the plane direction.
	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->penetration = penetration;
	contact->contactPoint = position - plane.direction * centreDistance;
	contact->setBodyData(sphere.body, NULL,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data)
{
	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Cache the sphere position
	Vector3 position = sphere.getAxis(3);

	// Find the distance from the plane
	real ballDistance =
		plane.direction * position -
		sphere.radius - plane.offset;

	if (ballDistance >= 0) return 0;

	// Create the contact - it has a normal in the plane direction.
	Contact* contact = data->contacts;
	contact->contactNormal = plane.direction;
	contact->penetration = -ballDistance;
	contact->contactPoint =
		position - plane.direction * (ballDistance + sphere.radius);
	contact->setBodyData(sphere.body, NULL,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data)
{
	//Make sure we have contacts.
	if (data->contactsLeft <= 0)
	{
		return 0;
	}

	//cache the sphere positions.
	Vector3 positionOne = one.getAxis(3);
	Vector3 positionTwo = two.getAxis(3);

	//Find the vector between the objects.
	Vector3 midline = positionOne - positionTwo;
	real size = midline.magnitude();

	//See if it is large enough.
	if (size <= 0.0f || size >= one.radius * two.radius)
	{
		return 0;
	}

	//We manually create the normal, because we have the
	//size to hand.
	Vector3 normal = midline * (((real)1.0 / size));

	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = positionOne + midline * (real)0.5;
	contact->penetration = (one.radius + two.radius - size);
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

//static inline real transformToAxis(
//	const CollisionBox& box,
//	const Vector3& axis
//)
//{
//	return
//		box.halfSize.x * real_abs(axis * box.getAxis(0)) +
//		box.halfSize.y * real_abs(axis * box.getAxis(1)) +
//		box.halfSize.z * real_abs(axis * box.getAxis(2));
//}


/*
 * This function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline real penetrationOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCentre)
{
	// Project the half-size of one onto axis
	real oneProject = transformToAxis(one, axis);
	real twoProject = transformToAxis(two, axis);

	// Project this onto the axis
	real distance = real_abs(toCentre * axis);

	// Return the overlap (i.e. positive indicates
	// overlap, negative indicates separation).
	return oneProject + twoProject - distance;
}

static inline bool tryAxis(const CollisionBox& one, const CollisionBox& two, Vector3& axis, const Vector3& toCentre, unsigned index,

	// These values may be updated
	real& smallestPenetration,
	unsigned& smallestCase)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squaredMagnitude() < 0.0001) return true;
	axis.normalize();

	real penetration = penetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0)
	{
		return false;
	}

	if (penetration < smallestPenetration) 
	{
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

void fillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, const Vector3& toCentre, CollisionData* data, unsigned best, real pen)
{
	// This method is called when we know that a vertex from
	// box two is in contact with box one.

	Contact* contact = data->contacts;

	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on
	// this axis.
	Vector3 normal = one.getAxis(best);
	if (one.getAxis(best) * toCentre > 0)
	{
		normal = normal * -1.0f;
	}

	// Work out which vertex of box two we're colliding with.
	// Using toCentre doesn't work!
	Vector3 vertex = two.halfSize;
	if (two.getAxis(0) * normal < 0) vertex.x = -vertex.x;
	if (two.getAxis(1) * normal < 0) vertex.y = -vertex.y;
	if (two.getAxis(2) * normal < 0) vertex.z = -vertex.z;

	// Create the contact data
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = two.getTransform() * vertex;
	contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

static inline Vector3 contactPoint(Vector3& pOne, Vector3& dOne, real oneSize, Vector3& pTwo, Vector3& dTwo, real twoSize,

	// If this is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	bool useOne)
{
	Vector3 toSt, cOne, cTwo;
	real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	real denom, mua, mub;

	smOne = dOne.squaredMagnitude();
	smTwo = dTwo.squaredMagnitude();
	dpOneTwo = dTwo * dOne;

	toSt = pOne - pTwo;
	dpStaOne = dOne * toSt;
	dpStaTwo = dTwo * toSt;

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (real_abs(denom) < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}



//unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data)
//{
//	//Make sure we have contacts.
//	if (data->contactsLeft <= 0)
//	{
//		return 0;
//	}
//
//	//Cache the sphere position.
//	Vector3 position = sphere.getAxis(3);
//
//	//Find the distance from the plane.
//	real ballDistance = plane.direction * position - sphere.radius - plane.offset;
//
//	if(ballDistance >= 0)
//	{
//		return 0;
//	}
//
//	//Cretea the contact; it has a normal in the plane direction.
//	Contact* contact = data->contacts;
//	contact->contactNormal = plane.direction;
//	contact->penetration = -ballDistance;
//	contact->contactPoint = position - plane.direction * (ballDistance + sphere.radius);
//	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);
//
//	data->addContacts(1);
//	return 1;
//}

//unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data)
//{
//	//Make sure we have contacts.
//	if (data->contactsLeft <= 0)
//	{
//		return 0;
//	}
//
//	//Cache the sphere posiiton.
//	Vector3 position = sphere.getAxis(3);
//
//	//Find the distance from the plane.
//	real centerDistance = plane.direction * position - plane.offset;
//
//	//Check if we are within radius.
//	if (centerDistance * centerDistance > sphere.radius * sphere.radius)
//	{
//		return 0;
//	}
//
//	//Check which side of the plane we are on.
//	Vector3 normal = plane.direction;
//	real penetration = -centerDistance;
//
//	if (centerDistance < 0)
//	{
//		normal *= -1;
//		penetration = -penetration;
//	}
//
//	penetration += sphere.radius;
//
//	//Create the contact; it has a normal in the plane diraction.
//	Contact* contact = data->contacts;
//	contact->contactNormal = normal;
//	contact->penetration = penetration;
//	contact->contactPoint = position - plane.direction * centerDistance;
//	contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);
//
//	data->addContacts(1);
//	return 1;
//}

unsigned CollisionDetector::boxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data)
{
	// Transform the point into box coordinates
	Vector3 relPt = box.transform.transformInverse(point);

	Vector3 normal;

	// Check each axis, looking for the axis on which the
	// penetration is least deep.
	real min_depth = box.halfSize.x - real_abs(relPt.x);
	if (min_depth < 0)
	{
		return 0;
	}

	normal = box.getAxis(0) * ((relPt.x < 0) ? -1 : 1);

	real depth = box.halfSize.y - real_abs(relPt.y);
	if (depth < 0)
	{
		return 0;
	}

	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.getAxis(1) * ((relPt.y < 0) ? -1 : 1);
	}

	depth = box.halfSize.z - real_abs(relPt.z);
	if (depth < 0)
	{
		return 0;
	}

	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.getAxis(2) * ((relPt.z < 0) ? -1 : 1);
	}

	// Compile the contact
	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = point;
	contact->penetration = min_depth;

	// Note that we don't know what rigid body the point
	// belongs to, so we just use NULL. Where this is called
	// this value can be left, or filled in.
	contact->setBodyData(box.body, NULL,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data)
{
	//Make sure we ahve contacts.
	if (data->contactsLeft <= 0)
	{
		return 0;
	}

	//Check for intersection.
	if (IntersectionTests::boxAndHalfSpace(box, plane))
	{
		return 0;
	}

	//We have an intersection, so find the intersection points. We can
	//make do with only checking vertices. If the box is resting on a plane
	//or an edge, it will be reported as four or two contact points.

	//Go through each combination of + and - for half-size.

	static real mults[8][3] = { {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1}, {1, -1, -1,}, {-1, -1, -1} };

	Contact* contact = data->contacts;
	unsigned contactsUsed = 0;
	
	for (unsigned i = 0; i < 8; i++)
	{
		//Calculate the position of each vertex.
		Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		vertexPos.componentProductUpdate(box.halfSize);
		vertexPos = box.transform.transform(vertexPos);

		//Calcualte the distance from the plane.
		real vertexDistance = vertexPos * plane.direction;

		//Compare this to the plane's distance;
		if (vertexDistance <= plane.offset)
		{
			//Create the contact data.
			//the contact point is halfway the direction by half the separation
			//distance and add the vertex location.
			contact->contactPoint = plane.direction;
			contact->contactPoint *= (vertexDistance - plane.offset);
			contact->contactPoint = vertexPos;
			contact->contactNormal = plane.direction;
			contact->penetration = plane.offset - vertexDistance;

			//Write the appropriate data.
			contact->setBodyData(box.body, NULL, data->friction, data->restitution);

			//Move on to the next contact.
			contact++;
			contactsUsed;

			if (contactsUsed == data->contactsLeft)
			{
				return contactsUsed;
			}
		}
	}

	data->addContacts(contactsUsed);
	return contactsUsed;
}

unsigned CollisionDetector::boxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data)
{
	// Transform the centre of the sphere into box coordinates
	Vector3 centre = sphere.getAxis(3);
	Vector3 relCentre = box.transform.transformInverse(centre);

	// Early out check to see if we can exclude the contact
	if (real_abs(relCentre.x) - sphere.radius > box.halfSize.x ||
		real_abs(relCentre.y) - sphere.radius > box.halfSize.y ||
		real_abs(relCentre.z) - sphere.radius > box.halfSize.z)
	{
		return 0;
	}

	Vector3 closestPt(0, 0, 0);
	real dist;

	// Clamp each coordinate to the box.
	dist = relCentre.x;
	if (dist > box.halfSize.x) dist = box.halfSize.x;
	if (dist < -box.halfSize.x) dist = -box.halfSize.x;
	closestPt.x = dist;

	dist = relCentre.y;
	if (dist > box.halfSize.y) dist = box.halfSize.y;
	if (dist < -box.halfSize.y) dist = -box.halfSize.y;
	closestPt.y = dist;

	dist = relCentre.z;
	if (dist > box.halfSize.z) dist = box.halfSize.z;
	if (dist < -box.halfSize.z) dist = -box.halfSize.z;
	closestPt.z = dist;

	// Check we're in contact
	dist = (closestPt - relCentre).squaredMagnitude();
	if (dist > sphere.radius * sphere.radius) return 0;

	// Compile the contact
	Vector3 closestPtWorld = box.transform.transform(closestPt);

	Contact* contact = data->contacts;
	contact->contactNormal = (closestPtWorld - centre);
	contact->contactNormal.normalize();
	contact->contactPoint = closestPtWorld;
	contact->penetration = sphere.radius - real_sqrt(dist);
	contact->setBodyData(box.body, sphere.body,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}

/*
 * This function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
//static inline real penetrationOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCenter)
//{
//	// Project the half-size of one onto axis
//	real oneProject = transformToAxis(one, axis);
//	real twoProject = transformToAxis(two, axis);
//
//	// Project this onto the axis
//	real distance = real_abs(toCenter * axis);
//
//	// Return the overlap (i.e. positive indicates
//	// overlap, negative indicates separation).
//	return oneProject + twoProject - distance;
//}

//static inline bool tryAxis(const CollisionBox& one, const CollisionBox& two, Vector3& axis, const Vector3& toCentre, unsigned index,
//
//	// These values may be updated
//	real& smallestPenetration,
//	unsigned& smallestCase
//)
//{
//	// Make sure we have a normalized axis, and don't check almost parallel axes
//	if (axis.squaredMagnitude() < 0.0001)
//	{
//		return true;
//	}
//	axis.normalize();
//
//	real penetration = penetrationOnAxis(one, two, axis, toCentre);
//
//	if (penetration < 0)
//	{
//		return false;
//	}
//
//	if (penetration < smallestPenetration) {
//		smallestPenetration = penetration;
//		smallestCase = index;
//	}
//	return true;
//}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
//#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)
//
//bool IntersectionTests::boxAndBox(const CollisionBox& one,const CollisionBox& two)
//{
//	// Find the vector between the two centres
//	Vector3 toCentre = two.getAxis(3) - one.getAxis(3);
//
//	return (
//		// Check on box one's axes first
//		TEST_OVERLAP(one.getAxis(0)) &&
//		TEST_OVERLAP(one.getAxis(1)) &&
//		TEST_OVERLAP(one.getAxis(2)) &&
//
//		// And on two's
//		TEST_OVERLAP(two.getAxis(0)) &&
//		TEST_OVERLAP(two.getAxis(1)) &&
//		TEST_OVERLAP(two.getAxis(2)) &&
//
//		// Now on the cross products
//		TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
//		TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
//		TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
//		TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
//		TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
//		TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
//		TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
//		TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
//		TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
//		);
//}
//#undef TEST_OVERLAP