/*

RigidBody Physics Engine ver_1.3, 2014.

RPEngine is RigidBody Dynamics for general purpose.

This engine referenced OpenSource below
1. Cyclone Engine - Ian Millington

Written by KWH. 2014-08-13.

*/

/* enable specialized version of RPEngine */
#define GET_OBJECTS_FORCE 
// if the app need force data, you can get from RigidBody 'linearForce' Vector3

/* Base components */
#include "Base/vector.h"
#include "Base/matrix.h"
#include "Base/quaternion.h"

/* Particle engine */
#include "pworld.h"

#include "particle.h"
#include "pfgen.h"
#include "pcontacts.h"
#include "plinks.h"

/* RigidBody engine */
#include "world.h"

#include "contacts.h"
#include "fgen.h"
#include "rbody.h"
// Collision system

#include "narrow_collision.h"

//additional components
#include "ContactGenerators/joint.h"
#include "ContactGenerators/staticJoint.h"


#if 0
	// not complete yet
	#include "broad_collision.h"
	// for engine users, some physical models
	// #include "Springkler.h"

#endif
