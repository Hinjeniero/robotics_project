/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
    innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/betaWorldArm.xml");
    timer.start(Period);
    return true;
}

void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState state;
  differentialrobot_proxy->getBaseState(state);
  innermodel->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); 
  try
  {
      switch(robotState) {
	case State::SEARCH: 
	  searchState();
	  break;
	case State::WAIT:  
	  waitState();
	  break;
	case State::IDLE:	  
	  idleState();
	  break;
	case State::HANDLER:	  
	  handlerState();
	  break;
      }
  }catch(const exception e) {
    
  }
}

void SpecificWorker::searchState(){
    gotopoint_proxy->turn(1);
    if(targetActive){  
      robotState = State::WAIT;
      gotopoint_proxy->go("", target.x(), target.z(), 0);
      std::cout << "transformed  - "<<target.x() << " <-> tag id -" << target.z() <<endl;
    }
}

void SpecificWorker::waitState(){
  if (gotopoint_proxy->atTarget()){
    std::cout << "arrived at target wait" << endl;
    gotopoint_proxy->go("", 0, 0, 0);
    robotState = State::IDLE;
  }
}

void SpecificWorker::idleState(){
  if (gotopoint_proxy->atTarget()){
    std::cout << "arrived at target idle" << endl;
    current++;
    if (current > 13)
      current = 10; //TODO delete this later :) 
    gotopoint_proxy->stop();
    targetActive = false;
    robotState = State::SEARCH;
  }
}

/*
void SpecificWorker::newAprilTag(const tagsList &tags){
  for(auto t: tags)
  {
    if (robotState == State::SEARCH && t.id == current){
      std::cout << "Current - "<<current << " <-> tag id -" << t.id <<endl;
      target = innermodel->transform("world", QVec::vec3(t.tx, t.ty, t.tz), "rgbd");
      currentTag = t;
      targetActive = true;
    }
    std::cout << t.id << endl;
    std::cout << t.tx << " " << t.ty << " " << t.tz << endl;  
    std::cout << "*****************************************" << endl;
  }
}*/


void SpecificWorker::newAprilTag(const tagsList &tags){
  tag t = nearestAprilTag(tags);
  if (robotState == State::SEARCH){
    std::cout << "Current - "<<current << " <-> tag id -" << t.id <<endl;
    target = innermodel->transform("world", QVec::vec3(t.tx, t.ty, t.tz), "rgbd");
    currentTag = t;
    targetActive = true;
  }
  std::cout << t.id << endl;
  std::cout << t.tx << " " << t.ty << " " << t.tz << endl;  
  std::cout << "*****************************************" << endl;
}

tag SpecificWorker::nearestAprilTag(const tagsList &tags){
  //sin periferia, la mas cercana de las que vea
  //-----------------------------
  float dist = sqrt(pow(tags[0].tx, 2)+pow(tags[0].tz, 2));
  tag tmin = tags[0];
  for (auto t: tags)
  {
    float aux = sqrt(pow(t.tx, 2)+pow(t.tz, 2));
    if(aux < dist){
      dist = aux;
      tmin = t;
    }
  }
 return tmin;
}

void SpecificWorker::handlerState() {
  
}



