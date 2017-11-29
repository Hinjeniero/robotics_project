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
  	innermodel = new InnerModel("/home/hinjeniero/robocomp/files/innermodel/simpleworld.xml");
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
      }
  }catch(const exception e) {
    
  }
}

void SpecificWorker::searchState(){
  
  if(!buffer.empty()){
    if (buffer.getFirst().id == current) {
      tag t = buffer.getFirst();
      robotState = State::WAIT;
      gotopoint_proxy->go("", t.rx, t.rz, 0);
      return;
    }
  }
  gotopoint_proxy->turn(1);
}

void SpecificWorker::waitState(){
  if (gotopoint_proxy->atTarget()){
    current++;
    robotState = State::SEARCH;
    gotopoint_proxy->stop();
    buffer.deleteFirst();
  }
  if (buffer.getFirst().id < current)
    current = 0;
}

void SpecificWorker::newAprilTag(const tagsList &tags){
  for(auto t: tags)
  {
    buffer.putAtEnd(t);
    std::cout << t.id << endl;
    std::cout << t.tx << " " << t.ty << " " << t.tz << endl;  
    std::cout << "*****************************************" << endl;
  }
  
}






