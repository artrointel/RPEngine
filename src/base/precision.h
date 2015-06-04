/*
this precision.h contains :
1. RPEngine configuration.
2. RPEngine is using these common definitions below.
*/

#ifndef _RP_ENGINE_PRECISION_H
#define _RP_ENGINE_PRECISION_H

#include<float.h>

namespace RPEngine
{
	#ifdef _RP_PRECISION_FLOAT
		typedef float real;
		#define REAL_MAX	FLT_MAX
		#define real_sqrt	sqrtf
		#define real_pow	powf
		#define real_abs	fabsf
		#define real_sin	sinf
		#define real_cos	cosf
		#define real_pi		3.141592f
		#define EPSILON		0.0001f
	#else
		#define DOUBLE_PRECISION
		typedef double real;
		#define REAL_MAX	DBL_MAX
		#define real_sqrt	sqrt
		#define real_pow	pow
		#define real_abs	abs				
		#define real_sin	sin
		#define real_cos	cos
		#define real_pi		3.141592
		#define EPSILON		0.0001
	#endif
}
#endif
