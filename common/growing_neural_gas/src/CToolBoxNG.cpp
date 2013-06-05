/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include<CToolBoxNG.h>

CToolBoxNG::CToolBoxNG()
{
}

std::vector<double>
CToolBoxNG::normalizePoint3D(std::vector<double> point)
{
	std::vector<double> normalizedpoint;

	normalizedpoint.resize(point.size());
	double a;
	double sum = 0;

	for(unsigned int i = 0; i<point.size(); i++)
	{
		sum +=  (point[i] * point[i]);
	}

	a = sqrt(sum);

	for(unsigned int i = 0; i<point.size(); i++)
	{
		normalizedpoint[i] =  (point[i]/a);
	}

	return normalizedpoint;
}

float
CToolBoxNG::angleBetweenPoints(std::vector<double> point1, std::vector<double> point2)
{
	float angle = 0;
	if(point1.size()!=point2.size())
	{
		return angle;
	}

	std::vector<double> normalizedpoint1;
	std::vector<double> normalizedpoint2;

	normalizedpoint1 = normalizePoint3D(point1);
	normalizedpoint2 = normalizePoint3D(point2);

	double sum = 0;
	for(unsigned int i = 0; i<normalizedpoint1.size(); i++)
	{
		sum += normalizedpoint1[i] * normalizedpoint2[i];
	}

	angle = (acos(sum) *  180.0 / M_PI);

	return angle;
}
