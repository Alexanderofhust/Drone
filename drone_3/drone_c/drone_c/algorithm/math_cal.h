#ifndef __MATH_CAL_H__
#define __MATH_CAL_H__

//float LIMIT_MAX_MIN(float x, float max, float min)
//{
//	if(x > max)
//		x = max;
//	else 
//		x = min;
//	
//	return x;
//}
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))  

#define ABS(x) ((x)>0? (x):(-(x)))


#endif
