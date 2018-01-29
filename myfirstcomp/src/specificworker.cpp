/* *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
  inner = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorldArm.xml");
  timer.start(Period);
  return true;
}

void SpecificWorker::compute(){
  try
  {
      RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
      RoboCompDifferentialRobot::TBaseState state;
      differentialrobot_proxy->getBaseState(state);
      inner->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); //Updates the transform values according to the robot's actual location	    
      switch(robotState) {
	case State::IDLE: 
	  idleState();
	  break;
	case State::GOTO:	  
	  gotoState(ldata);
	  break;
	case State::BUG: 
	  bugState(ldata);
	  break;
	case State::PICK: 
	  pickingBox();
	  break;
	case State::RELEASE: 
	  releasingBox();
	  break;	  
	case State::END: 
	  endState();
	  break;
      }
  }
  catch(const Ice::Exception &ex)
  {
      std::cout << ex << std::endl;
  }
}
	  
void SpecificWorker::idleState(){ 
  if (!target.isEmpty()){
    RoboCompDifferentialRobot::TBaseState state;
    differentialrobot_proxy->getBaseState(state);
    target.setRobotPos(state.x, state.z);
    robotState = State::GOTO;
  }
}
void SpecificWorker::gotoState(RoboCompLaser::TLaserData ldata) {
  if(obstacle(ldata, leftAngle, rightAngle)){ /*Obstacle detected, left and right side*/
   differentialrobot_proxy->setSpeedBase(0, 0); //TODO Take into account the braking
   robotState = State::BUG;
   decideTurnDirection(ldata);
   return;
  }

  //All variables are needed to calculate distance
  std::pair<float, float> t = target.getTarget();
  QVec tR = inner->transform("base", QVec::vec3(t.first, 0, t.second), "world"); //Vector's source is robot's location, vector's end is the mouse pick
  float d = tR.norm2(); //Gets the distance, that equals the vector's module
  //If no exit conditions
  std::cout << "ACTUAL DISTANCE - " <<d<<endl;
  if(d < MINDISTANCE){
    robotState = State::END;
    target.setEmpty();
    std::cout << "d < MINDISTANCE" <<endl;
    return;
  }
  /*We do continue on the GOTO state*/
  float adv;
  float rot = atan2(tR.x(), tR.z());
  if (rot > MAXROT) rot = MAXROT;
  adv = MAXVEL * getSigmoid(d) * getGauss(rot, 0.3, 0.5); 
  differentialrobot_proxy->setSpeedBase(adv, rot);
}

void SpecificWorker::bugState (RoboCompLaser::TLaserData ldata){
  RoboCompDifferentialRobot::TBaseState state;
  differentialrobot_proxy->getBaseState(state);
  float rot = MAXROT/2 + MAXROT%2;
  float adv = MAXVEL/2 + MAXVEL%2; //* getGauss(rot, 0.3, 0.5);
  int margin = 5;
  if((targetAtSight(ldata) && !obstacle(ldata, maxLeftAngle, maxRightAngle)) || (angleWithTarget() && !obstacle(ldata, maxLeftAngle, maxRightAngle))){
    robotState = State::GOTO;
    return;
  }
  
  if(obstacle(ldata, leftAngle+margin, rightAngle-margin)){
    if(lastWall == Turn::RIGHT){
      differentialrobot_proxy->setSpeedBase(0, -rot);
      return;
    }
    differentialrobot_proxy->setSpeedBase(0, rot);
    return;
  }
  if(lastWall == Turn::RIGHT && !obstacle(ldata, maxLeftAngle, leftAngle)){
      differentialrobot_proxy->setSpeedBase(adv, rot);
      return;
  }
  if (lastWall == Turn::LEFT && !obstacle(ldata, rightAngle, maxRightAngle)){
    differentialrobot_proxy->setSpeedBase(adv, -rot);
    return;
  }
  differentialrobot_proxy->setSpeedBase(adv, 0); //else
}

bool SpecificWorker::targetAtSight(RoboCompLaser::TLaserData ldata)
{
   QPolygonF polygon;
   QVec r = inner->transform("world", "base");
   polygon << QPointF(r.x(), r.z());
   for (int i = 30; i < 70; i++ ){
     QVec lr = inner->laserTo("world", "laser", ldata[i].dist, ldata[i].angle);
     polygon << QPointF(lr.x(), lr.z());
   }
   polygon << QPointF(r.x(), r.z());
   QPointF p(target.getTarget().first, target.getTarget().second); 
   if(polygon.containsPoint( p , Qt::WindingFill))
      std::cout << "MYFIRSTCOMP - targetAtSight()" << endl;
      std::cout << "x = "<< target.getTarget().first << " y = "<< target.getTarget().second << endl;
      std::cout << "target is at sight, going" << endl;
   return  polygon.containsPoint( p , Qt::WindingFill );  
}

bool SpecificWorker::angleWithTarget () {
  std::pair<float, float> t = target.getTarget();
  QVec tR = inner->transform("base", QVec::vec3(t.first, 0, t.second), "world");
  if (abs(atan2(tR.x(), tR.z())) < angleLimit){
    return true;}
  return false;
}

void SpecificWorker::endState(){
  std::cout << "MYFIRSTCOMP - endState()" << endl;
  differentialrobot_proxy->setSpeedBase(0, 0);
  robotState = State::IDLE;
}

bool SpecificWorker::obstacle(RoboCompLaser::TLaserData ldata, int start, int end){
  for(int i=start; i<= end; i++){
    if(ldata[i].dist < threshold)
      return true;
  }
  return false;
}

/*Decides the better turn, checking where there is "more" obstacle, on the left or on the right (Preference = left)*/
void SpecificWorker::decideTurnDirection(RoboCompLaser::TLaserData ldata){
  int j=0;
  for(int i = 0; i <= middleAngle-leftAngle; i++){
    j++;
    if (ldata[leftAngle+i].dist < threshold){
      std::cout << ldata[leftAngle+i].dist << "is less than threshold" << endl;
      lastWall = Turn::RIGHT;
      return;
    }
    if (ldata[rightAngle-i].dist < threshold){
      std::cout << ldata[rightAngle-i].dist << "is less than threshold" << endl;
      lastWall = Turn::LEFT;
      return;
    }
  }
}

void SpecificWorker::printLaser(RoboCompLaser::TLaserData ldata, int start, int end){
  for(int i=start; i<=end; i++)
    std::cout << "Laser position " << i << " distance -> " << ldata[i].dist << endl;
}

float SpecificWorker::getGauss(float Vr, float Vx, float h){
  float lambda = -pow(Vx, 2)/log(h);
  return pow(EulerConstant, (-pow(Vr, 2)/lambda));
}

float SpecificWorker::getSigmoid(float distance){
  float x = distance / 100;  //In sigmoid function, the changes on the curve are around 1-2-3 in the X
  return x / (1 + abs(x)); //"fast" sigmoid function
  
}

void SpecificWorker::setPick(const RoboCompRCISMousePicker::Pick& pick)
{
  target.setTarget(pick.x, pick.z);
  robotState = State::IDLE;
  std::cout << "MYFIRSTCOMP - setPick()" << endl;
  std::cout << "Location x -> " << pick.x << " was chosen." << endl;
  std::cout << "Location z -> " << pick.z << " was chosen." << endl; 
}

void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha){
  std::cout << "MYFIRSTCOMP - go()" << endl;
  target.setTarget(x, y);
  robotState = State::IDLE;
  std::cout << "Location x -> " << x << " was chosen." << endl;
  std::cout << "Location z -> " << y << " was chosen." << endl;     
}

void SpecificWorker::turn (const float speed) {
  differentialrobot_proxy->setSpeedBase(0, speed);
}

bool SpecificWorker::atTarget() {
  std::pair<float, float> t = target.getTarget();
  QVec tR = inner->transform("base", QVec::vec3(t.first, 0, t.second), "world"); //Vector's source is robot's location, vector's end is the mouse pick
  float d = tR.norm2(); //Gets the distance, that equals the vector's module
  //If no exit conditions
  if(d < MINDISTANCE){
    std::cout << "MYFIRSTCOMP - Arrived at target." << endl;
    return true;
  }
  return false;
}

 
void SpecificWorker::stop(){
  std::cout << "MYFIRSTCOMP - stop()" << endl;
  differentialrobot_proxy->setSpeedBase(0, 0);
  robotState = State::END;
}

void SpecificWorker::pickingBox() {
  std::cout << "pickingBox()" << endl; 
}

void SpecificWorker::releasingBox() {
  std::cout << "releasingBox()" << endl; 
}
