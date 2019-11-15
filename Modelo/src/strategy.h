#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "navigation.h"

#define GOALKEEPER 0
#define DEFENDER 1
#define FAKE9 2
#define STRIKER 3 
 


using namespace std;
using namespace vss;

class Strategy{

private:

IStateReceiver *stateReceiver;
ICommandSender *commandSender;
IDebugSender *debugSender;
State state;
float robot_radius;
float pwm_const;
float distance_to_stop;
float limiarTheta , deltaLimiar ;
float kp , kd , l , k;
float vDelta , vMax, vMaxGol, vSet;
float last_phi1, last_phi2, last_phi3, last_phi4;
string cor;
float distGiro;
Point3f atkPoint;

// field campo;

int defenderLine, goalkepperLine;
int statusPos1,statusPos2,statusPos3;
bool statusAtkSituation;

Robots teamRobots[3];
BallState ball; 

vss::Path path;
vss::Debug debug;

public:
	Strategy();	
	navigation Navigation;

	void loop();
	void sendVelocities();
	void pathControl();

	float distancePoint(Point3f a, Point3f b);
	Point3f midpoint(Point3f a, Point3f b);
	float angleTwoPoints(Point3f a, Point3f b);
	float radian(Point3f a, Point3f b);
	float ajustaAngulo(float angle);

	 void decisions();
	 void initPosition(int);
	 void setStatusPos(int);
	 void PlotPath(Robots robot);
	 
	int striker_chanel, fake9_chanel, volante_chanel, defender_chanel, goalkepper_chanel;
	int porrada_situation;
	
	void generic(Robots robo,Point3f a);
    void striker(Robots robo);
	void fake9(Robots robo);
	void volante(Robots robo);
	void defender(Robots robo);
	void goalkepper(Robots robo);
	
	void rotate(Robots);
	void atkSituation(Robots);

    void moverGeneric(Robots);
	void moverStriker(Robots);
	void moverDefender(Robots);
	void moverGoalkepper(Robots);
    void moverFake9(Robots);
	void moverVolante(Robots, Robots meta);
};

#endif // _STRATEGY_H_