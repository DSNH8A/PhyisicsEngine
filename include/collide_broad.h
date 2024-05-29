#pragma once

#include "core.h"
#include <vector>

#define PI 3.14

/*
* Stores the potential contact to check later.
*/
struct PotentialContact
{
	/*
	* Holds the bodies that might be in contact.
	*/
	RigidBody* body[2];

};

/*
* Represents a bounding sphere that can be tsted for overlap.
*/
class BoundingSphere
{
	Vector3 center;
	real radius;

public:
	/*
	* Creates a new bounding sphere at the given center and radius.
	*/
	BoundingSphere(const Vector3& center, real radius);

	/*
	* Creates a bounding sphere to enclose the two given bounding
	* spheres.
	*/
	BoundingSphere(const BoundingSphere& one, const BoundingSphere& two);

	/*
	* Checks whther the bounding sphere overlaps with the other given
	* bounding sphere.
	*/
	bool overlaps(const BoundingSphere* other) const;

	/*
	* Reports how much this bounding sphere would have to grow
	* by to incorporate the given bounding sphere. Note that this
	* calculation returns a value not in any particular units (i.e.
	* its not a volume growth). In fact the best implementation
	* takes into account the growth in surface area (after the
	* Goldsmith-Salmon algorithm for tree construction).
	*/
	real getGrowth(const BoundingSphere& other) const;

	/*
	* Returns the volume of this bounding volume. This is used
	* to calculate how to recurse into the bounding volume tree.
	* For a bounding sphere it is a simple calculation.
	*/
	real getSize() const
	{
		return ((real)1.333333) * PI * radius * radius * radius;
	}
};

/*
* A template class for nodes in a bounding volume hierarchy. This
* class uses a binary tree to store the bounding volumes.
*/
template<class BoundingVolumeClass>
class BVHNode
{
public:
	/*
	* Holds the child nodes of this node.
	*/
	BVHNode* children[2];

	/*
	* Holds a single bounding volume encompassing all the
	* descendants of this node.
	*/
	BoundingVolumeClass volume;

	/*
	* Holds the rigid body at this node of hierarchy.
	* Only leaf nodes can have a rigid body defined (see isLeaf).
	* Note that it is possible  to rewrite the algorithms in this 
	* class to handke objects at all levels of the hirarchy,
	* but  the code provided ignores this vetor unless firtChild
	* is NULL.
	*/
	RigidBody* body;

	/*
	* Holds the node immidiately above us in the tree.
	*/
	BVHNode* parent;

	BVHNode(BVHNode* parent, const BoundingVolumeClass& volume, RigidBody* body = NULL)
		: parent(parent), volume(volume), body(body)
	{
		children[0] = children[1] = NULL;
	}

	/*
	* Checks whether this node is a the bottom of the hierarchy.
	*/
	bool isLeaf() const
	{
		return (body != NULL);
	}

	/*
	* Checks the potential contacts from this node downward in
	* the hiaerarchy, writing them to the given array (up to the)
	* given limit). Returns the number of potential contacts it
	* found.
	*/
	unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) const;


	/*
	* Inserts the given rigi body, with the given bounding volume,
	* into the hierarchy. This may involve the creation of
	* further bounding volume nodes.
	*/
	void insert(RigidBody* body, const BoundingVolumeClass& volume);

	/*
	* Deletes this node, removing it first from the hierarchy,
	* along with its associated rigid body and child nodes. This
	* method deletes the node and all its children (but obviously
	* not the rigid bodies). This also has the effect of deleteing
	* the sibling of this jode, and changing the parent node so
	* that it contains the data currently in the system. Finally,
	* it ofrces the hierarchy above the current node to reconsider
	* its bounding volume.
	*/
	~BVHNode();

	/*
	* Checks for overlapping between nodes in the hierarchy. Note
	* that any bounding volume should have an overlaps method implemented
	* that checks for overlapping with another object of its own type.
	*/
	bool overlaps(const BVHNode<BoundingVolumeClass>* other) const;

	/*
	* Checks the potential contacts between this node and the given
	* other node, writing them to the given array (up to the
	* given limit). Returns the number of potential contacts it
	* found.
	*/
	unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other, PotentialContact* contacts, unsigned limit) const;

	/*
	* For non-leaf nodes, this method recalculates the bounding volume
	* based on the bounding volumes of its children.
	*/
	void recalculateBoundingVolume(bool recurse = true);
};

//Note that because we are dealing with a template here, we 
//need to have the implemetations accessible to anything that
//imports this header.

template<class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass>* other) const
{
	return volume->overlaps(other->volume);
}

template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert(RigidBody* newBody, const BoundingVolumeClass& newVolume)
{
	//If we are a leaf,  then the only option is to spawn two
	//new children and place the new body in one.
	if (isLeaf())
	{
		//Child one is a copy of us.
		children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);

		//Child  two holds the new body.
		children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

		//And we now lose the body (we are no longer a leaf).
		this->body = NULL;

		//We need to recalcualte our bounding volume.
		recalculateBoundingVolume();
	}

	//Otherwise, we need to work out which child gets to keep
	//the inserted body. We give ot to whoever would grow the
	//the least to incorporate it.
	else
	{
		if (children[0]->volume.getGrowth(newVolume) < children[1].getGrowth(newVolume))
		{
			children[0]->insert(newBody, newVolume);
		}

		else
		{
			children[1]->insert(newBody, newVolume);
		}
	}
}

template<class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode()
{
	//If we dont have a parent, then we ignore the sibling
	//processing.
	if (parent)
	{
		//Find our sibling.
		BVHNode<BoundingVolumeClass>* sibling;

		if (parent->children[0] == this)
		{
			sibling = parent->children[1];
		}

		else
		{
			sibling = parent->children[0];
		}

		//Write its data to our parent.
		parent->volume = sibling->volume;
		parent->body = sibling->body;
		parent->children[0] = sibling->children[0];
		parent->children[1] = sibling->children[1];

		//Delete the sibling (we blank its parent and
		//children to avoid processing/deletenig them).
		sibling->parent = NULL;
		sibling->body = NULL;
		sibling->childreng[0] = NULL;
		sibling->children[1] = NULL;
		delete sibling;

		//Recalculate the parents bounding volume.
		parent->recalculateBoundingVolume();
	}

	//Delete our children (again, we remove their
	//parent data so we dont try to process their siblings
	//as they are deleted).
	if (children[0])
	{
		children[0]->parent = NULL;
		delete children[0];
	}

	if (children[1])
	{
		children[1]->parent = NULL;
		delete children[0];
	}
}

template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(bool recurse)
{
	if (isLeaf())
	{
		return;
	}

	// Use the bounding volume combining constructor.
	volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);

	// Recurse up the tree
	if (parent) 
	{
		parent->recalculateBoundingVolume(true);
	}
}

template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(PotentialContact* contacts, unsigned limit) const
{
	//Early out if we dont have the room for contacts, or
	//if we are a leaf node.
	if (isLeaf() || limit == 0)
	{
		return 0;
	}

	//Get the potential contacts of one of four children with
	//the other.
	return children[0]->getPotentialContactsWith(children[1], contacts, limit);
}

template<class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(const BVHNode<BoundingVolumeClass>* other, PotentialContact* contacts, unsigned limit) const
{
	//Early out if we dont overlap or if we have no room
	//to report contacts.
	if (!overlaps(other) || limit == 0)
	{
		return 0;
	}
	
	//If we are both at leaf node, then we have a potential contact.
	if (isLeaf() && other->isLeaf())
	{
		contacts->body[0] = body;
		contacts->body[1] = other->body;
		return 1;
	}

	//Determine which node to descend the other into. If either is
	//a leaf, then we descend the other. If both are branches,
	//then we use the one with the largest size.
	if (other->isLeaf() || (!isLeaf()) && volume->getsize() >= other->volume->getSize())
	{
		//Recurse into self.
		unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

		//Check that we have enough slots to do the other side too.
		if (limit > count)
		{
			return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
		}

		else
		{
			return count;
		}
	}

	else
	{
		//Recurs into the other node.
		unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

		//Check that we have enough slots to do the ohter side too.
		if (limit > count)
		{
			return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
		}

		else
		{
			return count;
		}
	}
}

//void FuckWithTheTree(Object object, Plane plane, BSPNode node)
//{
//
//	real res = (object.position - plane.position)* plane.direction;
//
//
//	//Check if the new position is in the childNode or 
//	//in the parent node. If any ot those true move the 
//	//object accordingly.
//
//	if (res > 0)
//	{
//		if (node.hasParent() == true)
//		{
//			real res = (object.position - node.plane.position) * plane.direction;
//
//			if (res > 0)
//			{
//				FuckWithTheTree(object.position, node.parent.plane, node.parent.node);
//			}
//
//			if (res < 0)
//			{
//				node.parent.objects.add();
//			}
//
//		}
//		else
//		{
//			return;
//		}
//	}
//
//	if (res < 0)
//	{
//		if (node.haschild() == true)
//		{
//			real res = (object.position - Node.child[0].plane.position) * Node.child[0].plane.direction;
//
//			if (res < 0)
//			{
//
//				FuckWithTheTree(object.position, node.child[0].plane, node.child[0]);
//			}
//
//			if (res > 0)
//			{
//				node.objects.add(object);
//			}
//
//		}
//
//		else
//		{
//			return;
//		};
//
//	}
//}

//void broadPhaseCollisionDetection(BSPNode node)
//{
//	real res = (object.position - node.plane.position) * node.plane.direction;
//
//	for (BSPChild i = node.front.objects.begin(); i < node.front.objects.begin(); i++)
//	{
//		real res = (ndoe.front.objects[i].position - node.plane.position)* node.plane.direction;
//
//		if(res > 0)
//		{
//			continue;
//		}
//
//		if (res < 0)
//		{
//			node.front.objects.remove(i);
//			node.back.objects.add(i);
//		}
//	}
//
//	for (BSPChild i = node.back.objects.begin; i < node.back.objects.end(); i++)
//	{
//		real res = (node.back.objects[i].position - node.plane.position) * node.plane.direction;
//
//		if(res < 0)
//		{
//			continue;
//		}
//
//		if(res > 0 )
//		{
//			node.back.objects.remove(i);
//			node.front.objects.add(i);
//		}
//
//		for (BSPObject j =  i+ 1; j < node.back.objects.end(); j++)
//		{
//			if (node.back.objects[i].overlaps(objects[j]))
//			{
//				collideNarrow;
//			}
//		}
//
//		for (BSPObjects k = i + 1; k < node.front.objects.end(); k++)
//		{
//			if (node.front.objects[i].overlaps(objects[k]))
//			{
//				collideNarrow;
//			}
//		}
//	}
//}

/*void useOneList(BSPNode node, Object object)
{
	real res = (object.position - node.plane.position) * node.plane.direction;

	if (node.hasChild() == false)
	{
		node.objects.add(object);
	}

	if (res > 0)
	{
		useOneList(node.front, object);
	}

	if (res < 0)
	{
		useOneList(node.back, object);
	}
}*/


//struct Plane
//{
//	Vector3 position;
//	Vector3 direction;
//	Object objects; //list
//};
//
//enum BSPChildType
//{
//	NODE, 
//	OBJECTS
//};
//
//struct BSPChild
//{
//	BSPChildType type;
//
//	union 
//	{
//		BSPNode* node;
//		BSPOjbectSet objects; //object list
//	};
//};
//
//struct BSPNode
//{
//	Plane plane;
//	BSPChild front;
//	BSPChild back;
//};