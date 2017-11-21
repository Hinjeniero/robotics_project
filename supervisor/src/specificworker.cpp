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
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
  try
  {
      RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
      RoboCompDifferentialRobot::TBaseState state;
      differentialrobot_proxy->getBaseState(state);
      inner->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); //Updates the transform values according to the robot's actual location	    
      //std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
      switch(robotState) {
	case State::SEARCH: 
	  searchState();
	  break;
	case State::WAIT:	  
	  gotoState(ldata);
	  break;
      }
}

void SpecificWorker::searchState(){
  differentialrobot_proxy->setSpeedBase(0, MAXROT);
}

void SpecificWorker::waitState(){
  differentialrobot_proxy->setSpeedBase(0, 0);
}

void SpecificWorker::newAprilTag(const tagsList &tags){
  for(auto t: tags)
  {
    std::cout << t.id << endl;
    std::cout << t.tx << " " << t.ty << " " << t.tz << endl;  
    std::cout << "*****************************************" << endl;
  }
  
}






