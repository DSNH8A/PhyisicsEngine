

#include "collide_broad.h"

BoundingSphere::BoundingSphere(const BoundingSphere& one, const BoundingSphere& two)
{
	Vector3 centerOffset = two.center - one.center;
	real distance = centerOffset.squaredMagnitude();
	real radiusDiff = two.radius - one.radius;

	//Check whther the larger sphere encloses the small one.
	if (radiusDiff * radiusDiff >= distance)
	{
		if (one.radius > two.radius)
		{
			center = one.center;
			radius = one.radius;
		}

		else
		{
			center = two.center;
			radius = two.radius;
		}
	}

	//Ohterwise we nee to work with partially overlapping spheres.
	else 
	{
		distance = real_sqrt(distance);
		radius = (distance + one.radius + two.radius) * ((real)0.5);

		//The new center is based on one's center, moved toward
		//two's center by an amount proportional to the spheres
		//radii;
		center = one.center;

		if (distance > 0)
		{
			center += centerOffset * ((radius - one.radius) / distance);
		}
	}
}

bool BoundingSphere::overlaps(const BoundingSphere* other) const
{
	real distanceSquared = (center - other->center).squaredMagnitude();
	return distanceSquared < (radius + other->radius) * (radius + other->radius);
}