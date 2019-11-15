
 #include "strategy.h"

    Strategy::Strategy()
     {
        stateReceiver = new StateReceiver();
        commandSender = new CommandSender();
        debugSender = new DebugSender();

        stateReceiver->createSocket();

        // cor = "Yellow";
        // commandSender->createSocket(TeamType::Yellow);
        // debugSender->createSocket(TeamType::Yellow);
        // Navigation.setSide("Right");

         cor = "Blue";
         commandSender->createSocket(TeamType::Blue);
         debugSender->createSocket(TeamType::Blue);
    Navigation.setSide("Right");

    robot_radius = 8.0;
    distGiro = 8.2;
    defenderLine = 50;
    goalkepperLine = 10;
    vMaxGol = 120;
        statusPos1 = 0;
        statusPos2 = 0;
        statusPos3 = 0;
        pwm_const = 100.0;
        limiarTheta = 90 , deltaLimiar = 10;
        kp = 30, kd = 0.01 , l = 0.04;
        vMax = 50.0, vDelta = 0.6*vMax, vMaxGol = 0.5, vSet = 0.5;
        k = (7/vMax);
        striker_chanel = 0;
        fake9_chanel = 1;
        volante_chanel = 2;
        goalkepper_chanel = 2;
        defender_chanel = 2;
        porrada_situation = 0;
        last_phi1 = 0;

        for(int i = 0; i < 3; i++)
             teamRobots[i] = Robots(i,Point3f(0,0,0),0, 0,Vels(0,0),Point3f(0,0,0),-1);

        ball = BallState(Point3f(state.ball.x, state.ball.y, 0),Point3f(state.ball.speedX, state.ball.speedY, 0));
     }

     void Strategy::loop()
     {
         state = stateReceiver->receiveState(FieldTransformationType::None);

         for(int i = 0; i < 3; i++)
         {
             if(cor == "Yellow")
             {
                 teamRobots[i].pos =  Point3f(fabs(state.teamYellow.at(i).x - 170),fabs(state.teamYellow.at(i).y - 130),0);
                 teamRobots[i].angle = ajustaAngulo(state.teamYellow.at(i).angle - 180);
                 ball = BallState(Point3f(fabs(state.ball.x - 170), fabs(state.ball.y - 130), 0),Point3f(-state.ball.speedX, -state.ball.speedY, 0));
             }
             else if(cor == "Blue")
             {
                teamRobots[i].pos =  Point3f(state.teamBlue.at(i).x,state.teamBlue.at(i).y,0);
                teamRobots[i].angle = ajustaAngulo(state.teamBlue.at(i).angle);
                ball = BallState(Point3f(state.ball.x, state.ball.y, 0),Point3f(state.ball.speedX, state.ball.speedY, 0));
             }
         }
         decisions();
         sendVelocities();
         pathControl();
     }

    void Strategy::sendVelocities(){
        Command command;
        for(int i = 0; i < 3; i++)
        {
            // command.commands.push_back(WheelsCommand(10,-10));
            command.commands.push_back(WheelsCommand(teamRobots[i].vels.left, teamRobots[i].vels.right));
        }

        commandSender->sendCommand(command);
    }


    void Strategy::pathControl()
    {
        vss::Debug debug;
        float x[3],y[3], x1,x2,y1,y2;
        for(unsigned int i = 0 ; i < 3 ; i++){
            if(cor == "Yellow")
            {
                x[i] = fabs(teamRobots[i].goal.x - 170);
                y[i] = fabs(teamRobots[i].goal.y - 130);
            }
            else if(cor == "Blue")
            {
                x[i] = teamRobots[i].goal.x;
                y[i] = teamRobots[i].goal.y;
            }
            debug.finalPoses.push_back(Pose(x[i] , y[i] , 0));
            vss::Path path;
            // cout << "Inicio" << endl;
            for(unsigned int k = 1 ; k < Navigation.plotThePhi[i].size() ; k++)
            {
                if(cor == "Yellow")
                {
                    x1 = fabs(Navigation.plotThePhi[i].at(k-1).x - 170);
                    y1 = fabs(Navigation.plotThePhi[i].at(k-1).y - 130);
                    x2 = fabs(Navigation.plotThePhi[i].at(k).x - 170);
                    y2 = fabs(Navigation.plotThePhi[i].at(k).y - 130);
                }
                else if(cor == "Blue")
                {
                    x1 = Navigation.plotThePhi[i].at(k-1).x;
                    y1 = Navigation.plotThePhi[i].at(k-1).y;
                    x2 = Navigation.plotThePhi[i].at(k).x;
                    y2 = Navigation.plotThePhi[i].at(k).y;
                }
                        path.points.push_back(Point(x1, y1));
                        path.points.push_back(Point(x2, y2));
            }
               debug.paths.push_back(path);
            // cout << "Fim" << endl;
        }

        debugSender->sendDebug(debug);
    }

    void Strategy::decisions()
    {
        /*Point3f a;
        a.x=60;
        a.y=60;
        a.z = 0;*/
        teamRobots[2].function = DEFENDER;
        defender(teamRobots[2]);
        PlotPath(teamRobots[2]);
        teamRobots[1].function = STRIKER;
        striker(teamRobots[1]);
        PlotPath(teamRobots[1]);
        teamRobots[0].function = GOALKEEPER;
        goalkepper(teamRobots[0]);
        PlotPath(teamRobots[0]);
        /*
        generic(teamRobots[1],a);
        a.x=80;
        a.y=70;
        a.z = 45;
        generic(teamRobots[2],a);
        a.x=50;
        a.y=40;
        a.z = 65;
        generic(teamRobots[0],a);
        */
    }
    //bool qlq=false; 

    /*void Strategy::generic(Robots robo,Point3f a)
    { 
         int i = robo.index;
         teamRobots[i].goal.x = a.x;
         teamRobots[i].goal.y = a.y;
         teamRobots[i].goal.z = a.z;
         teamRobots[i].thePhi = Navigation.hyperbolicSpiral2(robo, teamRobots[i].goal);
         moverGeneric(robo);
    }*/

    void Strategy::striker(Robots robo)
    {
        int i=robo.index; //definir o robô
        teamRobots[i].goal.x = ball.pos.x;     // a meta do robo na posiçõ x é a bola na posiçã x
        teamRobots[i].goal.y = ball.pos.y;     // a meta do robo na posiçõ y é a bola na posiçõ y
       // teamRobots[i].thePhi= angleTwoPoints(robo.pos, teamRobots[i].goal); //ângulo entre o robô e a bola
        Point3f angdetudo;
       /* int angulo;
            if (qlq==false)
            {
            for (int a=0; a<=130; a++) // calcular o angulo entre cada ponto e a bola
            {
                for (int b=0; b<=170; b++)
                {
                    angdetudo.x=a;
                    angdetudo.y=b;
                    angdetudo.z=0;
                    angulo = angleTwoPoints(angdetudo, ball.pos);
                    cout << angulo << " ";
                }
                cout << endl;
            }
            qlq=true;
            }*/
            teamRobots[i].thePhi = Navigation.hyperbolicSpiral2(robo, teamRobots[i].goal);
            moverStriker(robo);
    }


void Strategy::defender(Robots robo)
{
    int i = robo.index;

   if(ball.pos.x > Navigation.centroidDef.x + defenderLine + 4)
   {
       teamRobots[i].goal.x = defenderLine;
       teamRobots[i].goal.y = ball.pos.y;
   }else
   {
       if(ball.pos.y > Navigation.centroidDef.y + 35)
       {
           teamRobots[i].goal.x = Navigation.centroidDef.x + 4;
           teamRobots[i].goal.y = Navigation.centroidDef.y + 40;
       }else 
       {
           teamRobots[i].goal.x = Navigation.centroidDef.x + 4;
           teamRobots[i].goal.y = Navigation.centroidDef.y - 40;
       }
       if (teamRobots[i].goal.y>=124)
       teamRobots[i].goal.z=180;
       if (teamRobots[i].goal.y<=6)
       teamRobots[i].goal.z=0;
   }

   teamRobots[i].thePhi = angleTwoPoints(robo.pos,teamRobots[i].goal);
   moverDefender(robo);
}


void Strategy::goalkepper(Robots robo)

{

   int i = robo.index;

       if(ball.pos.y > Navigation.centroidDef.y + 35)
       {
           teamRobots[i].goal.x = Navigation.centroidDef.x + 5;
           teamRobots[i].goal.y = Navigation.centroidDef.y + 25;
       }else if(ball.pos.y < Navigation.centroidDef.y - 35)
       {
           teamRobots[i].goal.x = Navigation.centroidDef.x + 5;
           teamRobots[i].goal.y = Navigation.centroidDef.y - 25;
       }else
       {
           teamRobots[i].goal.x = Navigation.centroidDef.x + 5;
           teamRobots[i].goal.y = ball.pos.y;
       }
       

   teamRobots[i].thePhi = angleTwoPoints(robo.pos,teamRobots[i].goal);
   moverGoalkepper(robo);
}

    void Strategy::fake9(Robots robo)
    {
        // int i = robo.index;
        // for(int k = 0; k < 3; k++)
        // {
        //     teamRobots[k]
        // }
        
    }
 /*   void Strategy::moverGeneric(Robots robo)
    {
        int i = robo.index;
        int b = distancePoint(teamRobots[i].goal, robo.pos);
        float alpha = (teamRobots[i].thePhi * 180 / pi) - robo.angle;
         alpha = ajustaAngulo(alpha);

         float v,Vmax = 0.6,limiar = 90,w,kp=15,kd=0;
         if(fabs(alpha) < limiar)
         {
               v = -Vmax * (fabs(alpha)/limiar) + Vmax;
               w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }else
         {
              alpha = ajustaAngulo(alpha);
              v = Vmax * (fabs(alpha)/limiar) - Vmax;
              w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }
         last_phi1=alpha;
         if(b<=5)
         {
            v = 0;
            if(teamRobots[i].angle <= teamRobots[i].goal.z + 1 && teamRobots[i].angle >= teamRobots[i].goal.z - 1){
              w = 0;
              cout << "oi1";
            }
            else{
              w = 1;
              cout << "oi2";
              }              
         }
         cout << "Angulo :" <<teamRobots[i].angle << endl;
        teamRobots[i].vels.left=(v+w*l)*100;
        teamRobots[i].vels.right=(v-w*l)*100;

    }*/
    void Strategy::moverStriker(Robots robo)
    {
         int i = robo.index;
         float alpha = (teamRobots[i].thePhi * 180 / pi) - robo.angle;
         alpha = ajustaAngulo(alpha);

         float v,Vmax = 0.8,limiar = 90,w,kp=15,kd=0;
         if(fabs(alpha) < limiar)
         {
               v = -Vmax * (fabs(alpha)/limiar) + Vmax;
               w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }else
         {
              alpha = ajustaAngulo(alpha);
              v = Vmax * (fabs(alpha)/limiar) - Vmax;
              w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }
         last_phi1=alpha;
         teamRobots[i].vels.left=(v+w*l)*100;
         teamRobots[i].vels.right=(v-w*l)*100;
    }

    void Strategy::moverDefender(Robots robo)
    {
         int i = robo.index;
         int distmeta_robo = distancePoint(teamRobots[i].goal, robo.pos);
         int distbola_robo = distancePoint(teamRobots[i].pos, ball.pos);
         float alpha = (teamRobots[i].thePhi) - robo.angle;
         alpha = ajustaAngulo(alpha);

         float v,Vmax = 0.8,limiar = 90,w,kp=15,kd=0;
         if(fabs(alpha) < limiar)
         {
               v = -Vmax * (fabs(alpha)/limiar) + Vmax;
               w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }else
         {
              alpha = ajustaAngulo(alpha);
              v = Vmax * (fabs(alpha)/limiar) - Vmax;
              w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }
         last_phi1=alpha;

         if(distmeta_robo<=5)
         {
            v = 0;
            w = 0;              
         }

         teamRobots[i].vels.left=(v+w*l)*100;
         teamRobots[i].vels.right=(v-w*l)*100;
         if(distbola_robo <= 30 && distmeta_robo<=5)
         {
             w=30;
         }
         if(distmeta_robo<=5)
         {
            v = 0;
            if(teamRobots[i].angle <= teamRobots[i].goal.z + 30 && teamRobots[i].angle >= teamRobots[i].goal.z - 30){
              w = 0;
            }
            else{
              w = 30;
              }              
         }
         teamRobots[i].vels.left=(v+w*l)*100;
         teamRobots[i].vels.right=(v-w*l)*100;
    }

    void Strategy::moverGoalkepper(Robots robo)
    {
         int i = robo.index;
         int distmeta_robo = distancePoint(teamRobots[i].goal, robo.pos);
         int distbola_robo = distancePoint(teamRobots[i].pos, ball.pos);
         float alpha = (teamRobots[i].thePhi) - robo.angle;
         alpha = ajustaAngulo(alpha);

         float v,Vmax = 0.8,limiar = 90,w,kp=15,kd=0;
         if(fabs(alpha) < limiar)
         {
               v = -Vmax * (fabs(alpha)/limiar) + Vmax;
               w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }else
         {
              alpha = ajustaAngulo(alpha);
              v = Vmax * (fabs(alpha)/limiar) - Vmax;
              w = (kp * (alpha / 180)) + (kd*(alpha - last_phi1)); 
         }
         last_phi1=alpha;
         if(distmeta_robo<=5)
         {
            v = 0;
            w = 0;              
         }
         teamRobots[i].vels.left=(v+w*l)*100;
         teamRobots[i].vels.right=(v-w*l)*100;
         if(distbola_robo <= 30 && distmeta_robo<=5)
         {
             w=30;
         }
         teamRobots[i].vels.left=(v+w*l)*100;
         teamRobots[i].vels.right=(v-w*l)*100;
    }

    void Strategy::rotate(Robots robo){
        int i = robo.index;

        if(distancePoint(robo.pos,ball.pos) < distGiro && (robo.pos.y < 20 || robo.pos.y > 110))
        {
            if(robo.pos.y < 20){
                teamRobots[i].vels.right = -150;
                teamRobots[i].vels.left = 150;
            }else if( robo.pos.y > 110){
                teamRobots[i].vels.right = 150;
                teamRobots[i].vels.left = -150;
            }
        }

     }

    void Strategy::atkSituation(Robots robo)
    {
        int i = robo.index;
        Point3f min,max;
        float theta,alpha;
    float v;
    float w;
        float dist = distancePoint(robo.pos,ball.pos);

        float vAtk = 150;
        float vDeltaAtk = 0.6*vAtk;
        float robotAngle = robo.angle;
        float angBallRobot = angleTwoPoints(robo.pos,ball.pos);
        angBallRobot = ajustaAngulo(angBallRobot);
        // cout << "r: " << robotAngle << endl;
        // cout << "a: " << angBallRobot << endl;
        if (dist < 12)
        {
            theta = Navigation.getAngleCpu()*(180/pi);
            alpha = ajustaAngulo(theta) - ajustaAngulo(robo.angle);
            alpha = ajustaAngulo(alpha);
                    //PID
            if (fabs(alpha) <= limiarTheta ){
                v = -vDeltaAtk*fabs(alpha)/limiarTheta + vAtk;
                w = kp*alpha/180 + kd*(alpha - last_phi1);
                limiarTheta = 90 - deltaLimiar;
            }
            else{
                alpha = ajustaAngulo(alpha+180);
                v = vDeltaAtk*fabs(alpha)/limiarTheta - vAtk;
                w = kp*alpha/180 + kd*(alpha - last_phi1);
                limiarTheta = 90 + deltaLimiar;
            }
            last_phi1 = alpha;

            if ((fabs(robotAngle) < 90 &&  fabs(robotAngle - angBallRobot) < 30))
            {
                teamRobots[i].vels.right = (v - l*w)*pwm_const;
                teamRobots[i].vels.left = (v + l*w)*pwm_const;
                // cout << "Ataque Situation any point" << endl;
            }
            else if(fabs(robotAngle) > 90 &&  (fabs(robotAngle - angBallRobot) > 150 && fabs(robotAngle - angBallRobot) < 210))
            {
                teamRobots[i].vels.right = (v - l*w)*pwm_const;
                teamRobots[i].vels.left = (v + l*w)*pwm_const;
                // cout << "Ataque Situation" << endl;
            }
        }
    }

    // Plot

    void Strategy::PlotPath(Robots robot)
    {
        int indexRobot = robot.index;
        Navigation.plotThePhi[indexRobot].clear();
        float ang;
        Robots virtualRobot;
        virtualRobot.pos = robot.pos;
        int r = 1;
        int cont = 0;
        while((distancePoint(virtualRobot.pos,robot.goal) > 6)&&(cont < 230))//(fabs(virtualRobot.pos.x - goal.x) > 1) && (fabs(virtualRobot.pos.y - goal.y) > 1))
        {
            Navigation.AddPlotPoint(virtualRobot.pos, indexRobot);
            switch (teamRobots[indexRobot].function){
            case GOALKEEPER:
                ang = angleTwoPoints(virtualRobot.pos,robot.goal)*pi/180;
                break;
            case DEFENDER:
                ang = angleTwoPoints(virtualRobot.pos,robot.goal)*pi/180;
                break;
            case STRIKER:
                ang = (ajustaAngulo(Navigation.UnivectorFieldPlot(virtualRobot, robot.goal, Point3f(0,0,0))*180/pi)*pi/180);
                break;
            case FAKE9:
                Robots killer = teamRobots[indexRobot];

                if (teamRobots[0].function == STRIKER)
                    killer = teamRobots[0];
                else if (teamRobots[1].function == STRIKER)
                    killer = teamRobots[1];
                else if (teamRobots[2].function == STRIKER)
                    killer = teamRobots[2];

                ang = Navigation.repulsiveMath(virtualRobot, killer.pos)*pi/180;
                break;
            }
            
            virtualRobot.pos.x = virtualRobot.pos.x + r*cos(ang);
            virtualRobot.pos.y = virtualRobot.pos.y + r*sin(ang);
                    // cout << "Ang: " << ang*180/pi << endl;
                    // cout << "VX: " << virtualRobot.pos.x << "VY: " << virtualRobot.pos.y << endl;
                    // cout << "RX: " << robot.pos.x << "RY: " << robot.pos.y << endl;
                    // cout << "GX: " << robot.goal.x << "VY: " << robot.goal.y << endl;
            cont++;
        }
        // cout << "End: " << cont << endl;
    }

//Funções Uteis

float Strategy::distancePoint(Point3f a, Point3f b)
{
    return sqrt(((a.x - b.x)*(a.x - b.x)) + ((a.y - b.y)*(a.y - b.y)));
}

Point3f Strategy::midpoint(Point3f a, Point3f b)
{
    return Point3f(((a.x + b.x) / 2.0), ((a.y + b.y) / 2.0), 0.0);
}

float Strategy::angleTwoPoints(Point3f a, Point3f b)
{
    return (atan2(b.y - a.y, b.x - a.x) * (180/M_PI));
}

float Strategy::radian(Point3f a, Point3f b)
{
    return atan2(a.y - b.y, a.x - b.x);
}

float Strategy::ajustaAngulo(float angle)
{
    if (angle < -180)
        angle = angle + 360;
    else if (angle > 180)
        angle = angle - 360;
    else
        angle = angle;
    return angle;
}