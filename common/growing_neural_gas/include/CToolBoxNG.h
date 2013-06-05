/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CTOOLBOX_NG_H
#define CTOOLBOX_NG_H

#include<iostream>
#include<vector>
#include<math.h>

#define _USE_MATH_DEFINES

class CToolBoxNG
{
public:
	CToolBoxNG();

	std::vector<double>	normalizePoint3D(std::vector<double> point);

	float angleBetweenPoints(std::vector<double> point1,std::vector<double> point2);
};
#endif
