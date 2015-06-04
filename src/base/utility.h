#ifndef _UTILITY_H
#define _UTILITY_H

/*
There are Simple utility functions.
*/

#include"utility.h"
#include"precision.h"

namespace RPEngine
{
	static inline bool IsEqual(real real_value_a, real real_value_b, real epsilon = EPSILON)
	{
		return (real_abs(real_value_a - real_value_b) < epsilon) ? true : false;
	}
}
#endif
