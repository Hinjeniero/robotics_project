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
      switch(robotState) {
	case State::SEARCH: 
	  searchState();
	  break;
	case State::WAIT:	  
	  waitState(ldata);
	  break;
      }
}

void SpecificWorker::searchState(){
  tag t = buffer.get()
 if (t.id == current) {
   robotState = State::WAIT;
 }
 myfirstcomp.turn(1);
}

void SpecificWorker::waitState(){
  robotState = State::SEARCH;
  myfirstcomp.turn(1);
  if (t.id == current){
  robotState = State::WAIT;    
  }
  //TODO connect to firstcomp and targetAt
}

void SpecificWorker::newAprilTag(const tagsList &tags){
  for(auto t: tags)
  {
    buffer.put(t);
    std::cout << t.id << endl;
    std::cout << t.tx << " " << t.ty << " " << t.tz << endl;  
    std::cout << "*****************************************" << endl;
  }
  
}






