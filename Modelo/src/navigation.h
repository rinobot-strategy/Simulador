#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "utils.h"
#undef Sucess

using namespace Eigen;
using namespace std;

class navigation{
public:
navigation();
~navigation();

string side;
Point3f centroidAtk, centroidDef; 
//float hyperbolicSpiral(Robots robo, Point3f goal);
void UnivectorField(Robots robo, Point3f meta, Point3f enemy);
void setThetaDir(float);
float GaussianFunc(float r);
float repulsiveAngle(float y, float x, Point3f pos);
//float tangencialRepulsive(Robots robo, Point3f meta, Point3f obstaculo, float r);
//float repulsiveSpiral(Robots robo, Point3f enemy, Point3f meta);
float thePhi, thetaDir, omega;
void setSide(string);
void fakeCph(Robots robo, Point3f meta);
float getAngleCpu();
float getAngle();
float repulsiveMath(Robots robo, Point3f obj);
Point3f metaFakeCPH;
float distancePoint(Point3f a, Point3f b);
Point3f midpoint(Point3f a, Point3f b);
float angleTwoPoints(Point3f a, Point3f b);
float radian(Point3f a, Point3f b);

void AddPlotPoint(Point3f point, int i);
vector<Point3f> plotThePhi[3];
float UnivectorFieldPlot(Robots robo, Point3f meta, Point3f enemy);
double hyperbolicSpiral(Robots robo,Point3f goal);
float hyperbolicSpiral2(Robots robo,Point3f goal);

};

#endif