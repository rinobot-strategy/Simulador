#include "navigation.h"

navigation::navigation() : side("right")
{
}
navigation::~navigation() {}

float navigation::repulsiveAngle(float x, float y, Point3f pos)
{
  float alpha;
  if (x - pos.x < 0)
    alpha = atan((y - pos.y) / (x - pos.x)) + pi;
  else
    alpha = atan((y - pos.y) / (x - pos.x));

  return alpha;
}

/*float navigation::hyperbolicSpiral(Robots robo, Point3f goal)
{
  float thetaUp, thetaDown, rhoUp, rhoDown;
  float phi;
  float de = 7, Kr = 10;
  float gSize;
  if (robo.pos.x > goal.x)
    gSize = 0;
  else
    gSize = 3;

  Vector3d p(robo.pos.x, robo.pos.y, 1), ph(0, 0, 0);

  MatrixXd m_trans(3, 3), m_rot(3, 3);
  m_trans << 1, 0, -goal.x, 0, 1, -goal.y, 0, 0, 1;
  m_rot << cos(-thetaDir), -sin(-thetaDir), 0, sin(-thetaDir), cos(-thetaDir), 0, 0, 0, 1;

  ph = m_rot * m_trans * p;

  thetaUp = atan2((ph(1) - de - gSize), ph(0)) + thetaDir;
  thetaDown = atan2((ph(1) + de + gSize), ph(0)) + thetaDir;
  rhoUp = sqrt(pow(ph(0), 2) + pow((ph(1) - de - gSize), 2));
  rhoDown = sqrt(pow(ph(0), 2) + pow((ph(1) + de + gSize), 2));

  if (ph(1) > gSize)
    phi = thetaUp + pi * (2 - (de + Kr) / (rhoUp + Kr)) / 2;
  else if (ph(1) < -gSize)
    phi = thetaDown - pi * (2 - (de + Kr) / (rhoDown + Kr)) / 2;
  else
    phi = thetaDir;

  return phi;

  // float Kr = 5,phi;
  // float theta, rho, y_aux, yl, yr , phi_cw, phi_ccw;
  // float pl[2], pr[2], vec[2];
  // float de = 7;
  // Vector3d p(xi,yi,1),ph(0,0,0);

  // Matrix3d m_trans1(3,3),m_trans2(3,3),m_rot(3,3);
  // // Matriz para transladar a bola pra origem
  // m_trans1 << 1, 0, -meta.x, 0, 1, -meta.y, 0, 0, 1;
  // // Matriz para transladar a bola da origem pra posição original
  // m_trans2 << 1 ,0 ,meta.x, 0, 1, meta.y, 0, 0, 1;

  // m_rot << cos(-thetaDir),-sin(-thetaDir),0,sin(-thetaDir),cos(-thetaDir),0,0,0,1;

  // ph = m_trans2*m_rot*m_trans1*p;

  // pl[0] = ph(0);
  // pl[1] = ph(1) - de;
  // pr[0] = ph(0);
  // pr[1] = ph(1) + de;

  // y_aux = ph(1) - meta.y;

  // yl = y_aux + de;
  // yr = y_aux - de;

  // rho = sqrt(pow(pl[0]-meta.x,2)+pow(pl[1]-meta.y,2));
  // theta = atan2(pl[1]-meta.y,pl[0]-meta.x);

  // if (rho > de){
  //     phi_ccw = theta + pi*(2-((de+Kr)/(rho+Kr)))/2;
  // }else{
  //     phi_ccw = theta + pi*sqrt(rho/de)/2;
  // }

  // rho = sqrt(pow(pr[0]-meta.x,2)+pow(pr[1]-meta.y,2));
  // theta = atan2(pr[1]-meta.y,pr[0]-meta.x);

  // if (rho > de){
  //     phi_cw = theta - pi*(2-((de+Kr)/(rho+Kr)))/2;
  // }else{
  //     phi_cw = theta - pi*sqrt(rho/de)/2;
  // }

  // vec[0] = (yl*cos(phi_ccw) - yr*cos(phi_cw))/(2*de);
  // vec[1] = (yl*sin(phi_ccw) - yr*sin(phi_cw))/(2*de);

  // phi = atan2(vec[1],vec[0]) + thetaDir;
  // return phi;
}
*/
//void navigation::UnivectorField(Robots robo, Point3f meta, Point3f enemy)
//{
  // float k0 = 40;
  // float d_min = 20;   // Raio de Influencia do repulsivo
  // //float r = 15;   // Raio de Influencia do repulsivo
  // float norma_s,fih_AUF,fih_TUF;
  // float d = 2;//distancePoint(robo.pos, enemy);  //distancia entre o robo e o obstaculo
  // Point3f s, enemy_vel, robo_vel, virtual_obj;
  // enemy_vel.x = 0;
  // enemy_vel.y = 0;
  // robo_vel.x = 0;
  // robo_vel.y = 0;
  // // cout << robo_vel.x << " - " << robo_vel.y << endl;

  // s.x = k0 * (0  - robo_vel.x); // Velocidade
  // s.y = k0 * (0 - robo_vel.y);

  // norma_s = sqrt(pow(s.x,2) + pow(s.y,2));

  //     if (d >= norma_s)
  //     {
  //         virtual_obj.x = enemy.x + s.x;
  //         virtual_obj.y = enemy.y + s.y;
  //     //    cout << "case 1" << endl;
  //     }
  //     else
  //     {
  //     //    cout << "case 2" << endl;
  //         virtual_obj.x = enemy.x + (d*s.x/norma_s);
  //         virtual_obj.y = enemy.y + (d*s.y/norma_s);
  //     }

  // if(virtual_obj.x <= enemy.x && virtual_obj.x >= robo.pos.x  ){
  // // std::cout << "EX:" << enemy.x << std::endl;
  // // std::cout << "EY:" << enemy.y << std::endl;
  // // std::cout << "RX:" << robo.pos.x << std::endl;
  // // std::cout << "RY:" << robo.pos.y << std::endl;
  // //std::cout << "X:" << virtual_obj.x << std::endl;
  // // std::cout << "Y:" << virtual_obj.y << std::endl;
  // }

  // fih_TUF = hyperbolicSpiral(robo.pos.y,robo.pos.x,meta);

  // // Repulsivo Inatel
  // //fih_AUF = repulsiveAngle(robo.pos.x,robo.pos.y,virtual_obj);

  // fih_AUF = repulsiveMath(robo, meta, enemy);

  // if (d <= d_min){
  //     thePhi = fih_AUF ;
  //   //  cout << "desvinsasasasd!" << endl;
  //   }
  //   else{
  //     // thePhi = fih_TUF*(1-Gaussian_Func(d - d_min)) + fih_AUF*Gaussian_Func(d - d_min);
  //   thePhi = fih_TUF;
  // //  cout << "follow" << endl;
  //   }
  //  Fim Repulsivo Inatel

  //  // Repulsive Tangencial and Spiral

  // fih_AUF = tangencial_repulsive(robo,meta,enemy,r);
  // fih_AUF = repulsiveSpiral(robo,enemy,meta);

  // if (d <= r){
  //     thePhi = fih_AUF ;
  //     }
  // else{
  //     thePhi = fih_AUF*Gaussian_Func(d - r) + fih_TUF*(1-Gaussian_Func(d - r));
  //  }
  // // Fim do Repulsive tangencial
  //  thePhi = fih_AUF;
 // float aux = hyperbolicSpiral(robo, meta);
 // thePhi = aux;
//}

float navigation::GaussianFunc(float r)
{
  float delta = 15;
  float G;
  G = pow(M_E, (-pow(r, 2) / (2 * pow(delta, 2))));
  //cout << endl << "G: " << G << endl;
  return G;
}

void navigation::setThetaDir(float val)
{
  // cout << "t_dir: " << thetaDir*(180/pi) << endl;
  thetaDir = val;
}

// float navigation::tangencialRepulsive(Robots robo, Point3f meta, Point3f enemy, float r){
//   float alpha,omega,zeta,dist_robo_obst;
//   int rot;
//   float R = 5;
//     omega = repulsiveAngle(robo.pos.x,robo.pos.y,meta);  // Angulo entre o robo e a bola
//     zeta = repulsiveAngle(enemy.x,enemy.y,meta);                  // Angulo entre o obstaculo e a bola

//    if(omega < 0 && zeta < 0){
//        if(zeta <= omega){
//            rot = -1;
//        }
//        else{
//            rot = 1;
//        }
//    }
//    else{
//        if(zeta <= omega){
//            rot = 1;
//        }
//        else{
//            rot = -1;
//        }
//    }

//   dist_robo_obst = distancePoint(robo.pos,enemy);

//   alpha = -atan(R/dist_robo_obst);
// //  cout << endl << "repulsive otario: " << alpha*180/pi << endl;
//   return alpha;
// }

float navigation::repulsiveMath(Robots robo, Point3f obj)
{
  float rot_angle = pi / 2;
  float k_const = 1, k_larg = 0.06; // k_larg: Quanto menor mais desvia [0.1 , 0.01]
  float m = (robo.goal.y - obj.y) / (robo.goal.x - obj.x);
  float norm;
  double psi;
  int a;
  Point3f vec_out, vec_tan, vec;
  MatrixXd rot(2, 2);
  MatrixXd vec_tan_aux(2, 1), vec_aux(2, 1);

  norm = sqrt(pow(obj.x - robo.pos.x, 2) + pow(obj.y - robo.pos.y, 2));
  k_const = k_larg * norm;

  if (obj.x <= robo.goal.x)
  {
    if (robo.pos.y - obj.y > m * (robo.pos.x - obj.x))
      a = -1;
    else
      a = 1;
  }
  else
  {
    if (robo.pos.y - obj.y > m * (robo.pos.x - obj.x))
      a = 1;
    else
      a = -1;
  }

  vec_out.x = robo.goal.x - robo.pos.x;
  vec_out.y = robo.goal.y - robo.pos.y;

  rot << cos(a * rot_angle), sin(a * rot_angle), -sin(a * rot_angle), cos(a * rot_angle);

  vec_aux << vec_out.x, vec_out.y;
  vec_tan_aux = rot * vec_aux;
  vec_tan.x = vec_tan_aux(0);
  vec_tan.y = vec_tan_aux(1);

  vec.x = vec_tan.x + k_const * vec_out.x;
  vec.y = vec_tan.y + k_const * vec_out.y;

  psi = atan2(vec.y, vec.x);

  return psi * 180 / pi;
}

// float navigation::repulsiveSpiral(Robots robo, Point3f enemy, Point3f meta){
// float d = distancePoint(robo.pos, enemy);  //distancia entre o robo e o obstaculo
// float angle = angleTwoPoints(robo.pos, enemy)*(pi/180);
// float Kr = 3, de = 5;
// float out,omega,zeta,kappa;
// float a_min,a_max;
// int rot;
// omega = repulsiveAngle(meta.x,meta.y,robo.pos);  // Angulo entre o robo e a bola
// zeta = repulsiveAngle(enemy.x,enemy.y,meta);                  // Angulo entre o obstaculo e a bola
// kappa = repulsiveAngle(robo.pos.x,robo.pos.y,enemy);
// a_max = omega + pi/6;
// a_min = omega - pi/6;
// //cout << endl << "Omega: " << omega*180/pi << " kappa: " << kappa*180/pi << "Limites ( " << a_min*180/pi << " , " << a_max*180/pi << " )" << endl;

// if(omega < 0 && zeta < 0){
//     if(zeta <= omega){
//         rot = -1;
//     }
//     else{
//         rot = 1;
//     }
// }
// else{
//     if(zeta <= omega){
//         rot = 1;
//     }
//     else{
//         rot = -1;
//     }
// }
// rot = 1;
// // if(a_min < kappa && kappa < a_max){
// if(rot == 1){
// if(fabs(d) > de){
//     out = angle + ((pi/2)*(2-((de+Kr)/(fabs(d+Kr)))));
// }else{
//     out = angle + ((pi/2)*(sqrt((fabs(d)/de))));
// }
// }else{
// if(fabs(d) > de){
//     out = angle - ((pi/2)*(2-((de+Kr)/(fabs(d+Kr)))));
// }else{
//     out = angle - ((pi/2)*(sqrt((fabs(d)/de))));
// }
// }
// // }else{
// //    out = omega;
// // }
// //cout << endl << out*(180/pi) << endl;
// return out;
//   return 0;
// }

void navigation::setSide(string val)
{
  if (val == "Right")
  {
    centroidAtk.x = 160;
    centroidAtk.y = 65;
    centroidDef.x = 10;
    centroidDef.y = 65;
  }
  else
  {
    centroidDef.x = 160;
    centroidDef.y = 65;
    centroidAtk.x = 10;
    centroidAtk.y = 65;
  }
}

float navigation::getAngleCpu()
{
  return thePhi;
}

float navigation::getAngle()
{
  return omega;
}

void navigation::fakeCph(Robots robo, Point3f meta)
{
  metaFakeCPH = meta;
  omega = angleTwoPoints(meta, robo.pos);
}

float navigation::distancePoint(Point3f a, Point3f b)
{
  return sqrt(((a.x - b.x) * (a.x - b.x)) + ((a.y - b.y) * (a.y - b.y)));
}

Point3f navigation::midpoint(Point3f a, Point3f b)
{
  return Point3f(((a.x + b.x) / 2.0), ((a.y + b.y) / 2.0), 0.0);
}

float navigation::angleTwoPoints(Point3f a, Point3f b)
{
  return (atan2(b.y - a.y, b.x - a.x) * (180 / M_PI));
}

float navigation::radian(Point3f a, Point3f b)
{
  return atan2(a.y - b.y, a.x - b.x);
}

void navigation::AddPlotPoint(Point3f point, int i)
{
  plotThePhi[i].push_back(point);
}

float navigation::UnivectorFieldPlot(Robots robo, Point3f meta, Point3f enemy)
{
  float aux = hyperbolicSpiral2(robo, meta);
  return aux;
}

double navigation::hyperbolicSpiral(Robots robo,Point3f goal)
{
  double thetaUp, thetaDown, rhoUp, rhoDown;
  double phi,Kr = 2,de = 10,gSize;
  cout<<"oi";
  if(robo.pos.x > goal.x) 
      gSize = 0;
  else
      gSize = 3;
  Vector3d p(robo.pos.x, robo.pos.y, 1), ph(0, 0, 0);

  MatrixXd m_trans(3, 3), m_rot(3, 3);
  m_trans << 1, 0, -goal.x, 0, 1, -goal.y, 0, 0, 1;
  m_rot << cos(-thetaDir), -sin(-thetaDir), 0, sin(-thetaDir), cos(-thetaDir), 0, 0, 0, 1;

  ph = m_rot * m_trans * p;

  thetaUp = atan2((ph(1) - de - gSize), ph(0)) + thetaDir;
  thetaDown = atan2((ph(1) + de + gSize), ph(0)) + thetaDir;
  rhoUp = sqrt(pow(ph(0), 2) + pow((ph(1) - de - gSize), 2));
  rhoDown = sqrt(pow(ph(0), 2) + pow((ph(1) + de + gSize), 2));

  if (ph(1) > gSize)
    phi = thetaUp + pi * (2 - (de + Kr) / (rhoUp + Kr)) / 2;
  else if (ph(1) < -gSize)
    phi = thetaDown - pi * (2 - (de + Kr) / (rhoDown + Kr)) / 2;
  else
    phi = thetaDir;

  return phi;
}

float navigation::hyperbolicSpiral2(Robots robo,Point3f goal)
{
  float phi, Kr = 3;
    float theta, rho, y_aux, yl, yr, phi_cw, phi_ccw;
    float pl[2], pr[2], vec[2], de=5;

    Vector3d p(robo.pos.x, robo.pos.y, 1), ph(0, 0, 0);

    Matrix3d m_trans1(3, 3), m_trans2(3, 3), m_rot(3, 3);
    // Matriz para transladar a bola pra origem
    m_trans1 << 1, 0, -goal.x, 0, 1, -goal.y, 0, 0, 1;
    // Matriz para transladar a bola da origem pra posição original
    m_trans2 << 1, 0, goal.x, 0, 1, goal.y, 0, 0, 1;
    //Matriz para Rotacionar o Campo para a para a orientação de thataDir
    m_rot << cos(-thetaDir), -sin(-thetaDir), 0, sin(-thetaDir), cos(-thetaDir), 0, 0, 0, 1;

    ph = m_trans2 * m_rot * m_trans1 * p;

    if(goal.y )
    pl[0] = ph(0);
    pl[1] = ph(1) - de;
    pr[0] = ph(0);
    pr[1] = ph(1) + de;
    y_aux = ph(1) - goal.y;
    yl = y_aux + de;
    yr = y_aux - de;
    rho = sqrt(pow(pl[0] - goal.x, 2) + pow(pl[1] - goal.y, 2));
    theta = atan2(pl[1] - goal.y, pl[0] - goal.x);

    if (rho > de)
    {
      phi_ccw = theta + pi * (2 - ((de + Kr) / (rho + Kr))) / 2;
    }
    else
    {
      phi_ccw = theta + pi * sqrt(rho / de) / 2;
    }

    rho = sqrt(pow(pr[0] - goal.x, 2) + pow(pr[1] - goal.y, 2));
    theta = atan2(pr[1] - goal.y, pr[0] - goal.x);

    if (rho > de)
    {
      phi_cw = theta - pi * (2 - ((de + Kr) / (rho + Kr))) / 2;
    }
    else
    {
      phi_cw = theta - pi * sqrt(rho / de) / 2;
    }

    vec[0] = (yl * cos(phi_ccw) - yr * cos(phi_cw)) / (2 * de);
    vec[1] = (yl * sin(phi_ccw) - yr * sin(phi_cw)) / (2 * de);

    phi = atan2(vec[1], vec[0]) + thetaDir;
    return phi;
}