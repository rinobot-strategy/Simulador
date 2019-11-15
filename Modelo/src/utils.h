#ifndef UTILS_H
#define UTILS_H

#include <Communications/StateReceiver.h>
#include <Communications/CommandSender.h>
#include <Communications/DebugSender.h>
#include "cstdlib"
#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>
#define pi M_PI

using namespace std;

struct Point3f
{
    float x, y, z;
 
    Point3f(){
        x = y = z = 0;
    };
 
    Point3f(float x, float y, float z){
        this->x = x;
        this->y = y;
        this->z = z;
    };
 
    Point3f(Point3f *b){
        x = b->x;
        y = b->y;
        z = b->z;
    };
 
    void show(){
        printf("Point3f(%f, %f, %f)\n", x, y, z);
    };
};

struct Vels
{
    float left, right;
 
    Vels(){
        left = right = 0;
    };
 
    Vels(float l, float r){
        this->left = l;
        this->right = r;
    };
 
    Vels(Vels *b){
        left = b->left;
        right = b->right;
    };
 
    void show(){
        printf("Vels(%f, %f)\n", left, right);
    };
};
 
struct Robots
{
    int index, function;
    Point3f pos, goal;
    float angle, thePhi;
    Vels vels;
 
    Robots(){
        index = 0;
        function = 0;
        angle = 0;
        thePhi = 0;
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        goal.x = 0;
        goal.y = 0;
        goal.z = 0;
        vels.right = 0;
        vels.left = 0;
    };
 
    Robots(int i, Point3f p, float a, float t, Vels v, Point3f g, int f){
        this->index = i;
        this->function = f;
        this->vels = v;
        this->pos = p;
        this->goal = g;
        this->angle = a;
        this->thePhi = t;
    };
 
    Robots(Robots *b){
        index = b->index;
        function = b->function;
        pos = b->pos;
        angle = b->angle;
        vels = b->vels;
        thePhi = b->thePhi;
        goal = b->goal;
    };
 
    void show(){
        cout << "Index: " << index << endl;
        cout << "Function: " << function << endl;
        cout << "Posição: " << endl;
        pos.show();
        cout << "Angle: " << angle << endl;
        cout << "ThePhi: " << angle << endl;
        cout << "Vels: " << endl;
        vels.show();
        cout << "Meta: " << endl;
        goal.show();
    };
    float GetFollowAngle()
    {
        return thePhi;
    };
};

struct BallState
{
    Point3f pos, vels;
 
    BallState(){
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        vels.x = 0;
        vels.y = 0;
    };
 
    BallState(Point3f p, Point3f v){
        this->vels = v;
        this->pos = p;
    };
 
    BallState(BallState *b){
        pos = b->pos;
        vels = b->vels;
    };
 
    void show(){
        pos.show();
        vels.show();
    };
};

#endif