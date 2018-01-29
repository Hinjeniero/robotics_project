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
    innermodel = new InnerModel("/home/ronniejd/robocomp/files/innermodel/betaWorldArm.xml");
    timer.start(Period);
    return true;
}

void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState state;
  try{
    differentialrobot_proxy->getBaseState(state);
  }catch(const Ice::Exception &ex) {
    std::cout << "Exception differentialrobot_proxy in supervisor - "<< ex <<endl; 
  
  }
 //Updating the robot coordinates in each compute turn
  innermodel->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); 
  getMarcas();
  
  try
  {
      switch(robotState) {
	case State::SEARCH:
	  try
	  {	
	    searchState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in searchState - "<< ex <<endl; 
	  }
	  break;
	case State::WAIT:
	  try
	  {	
	    waitState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in waitState - "<< ex <<endl; 
	  }
	  break;
	case State::IDLE:
	  try
	  {	
	    idleState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in idleState - "<< ex <<endl; 
	  }
	  break;
	case State::HANDLERPICK:
	  try
	  {	
	    handlerPickState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in handlerPickState - "<< ex <<endl; 
	  }
	  break;
	case State::HANDLERRELEASE:
	  try
	  {	
	    handlerReleaseState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in handlerReleaseState - "<< ex <<endl; 
	  }
	  break;
      }
  }catch(const Ice::Exception &ex) {
    std::cout << "Exception in switch robotState supervisor - "<< ex <<endl; 
  }
}

/*State to search for boxes turning indefinitely*/
void SpecificWorker::searchState(){    
    std::cout << "SEARCH - Buscando la id " << currentBox <<endl;
    gotopoint_proxy->turn(0.5); //Turning until it finds a box
    if(targetActive){ //Found the desired box in getMarcas 
      //std::cout << "SEARCH - De camino a target X = "<< target.x() << " Z = " << target.z() <<endl;      
      //gotopoint_proxy->go("", target.x(), target.z(), 0); //Going to the desired box location  
      //targetActive = false;
      robotState = State::WAIT;
    } 
}

/*State waiting for the subcomponent to move the robot until the target location*/
void SpecificWorker::waitState(){
  std::cout << "WAIT" << endl;
  if (!holdingBox)
    gotopoint_proxy->go("", target.x(), target.z(), 0); //Going to the desired box location  
    
  std::cout << "X= "<< target.x() << " Z= " << target.z() << endl;
  if (gotopoint_proxy->atTarget()){ //IF the robot arrived at the target (box)
    std::cout << "Print atTarget in WAIT()" << endl;
    if(!holdingBox){ //If is at the box (not holding it yet)
      robotState = State::HANDLERPICK;
      return;
    }      
    //After reaching the corner, has to drop the box
    std::cout <<" handler state" << endl;
    robotState = State::HANDLERRELEASE;
  }
}

/*State waiting for the subcomponent to move the robot to the starting location*/
void SpecificWorker::idleState(){
  std::cout << "IDLE state" << endl;
  std::cout << "Idle state, reached target, back at the beginning" << endl;
  targetActive = false; //Dont have a target anymore to go to
  if (!holdingBox)
    currentBox++; //Incrementing index, so the box we look for is the next one

  robotState = State::SEARCH; 
}

/*State manipulating the box with the arm*/
void SpecificWorker::handlerPickState() {
  cout << "PICKING BOX"<< endl;
  targetActive = false;
  switch(dropPlace){//Decides which corner will be the dropping place
      case 0:{
	std::cout << "case 0" << endl;
	//target = innermodel->transform("world", QVec::vec3(2000, 0, 2000), "base"); //Setting the target
	gotopoint_proxy->go("",2000,2000,0);
	holdingBox = true;
	targetActive =true;
	//std::cout << "camino a x= " << target.x() << " z= " << target.z() << endl;
	break;
      }
      case 1:
	gotopoint_proxy->go("",-2000,2000,0); //low left corner
	break;
      case 2:
	gotopoint_proxy->go("",-2000,-2000,0); //low right corner
	break;
      case 3:
	gotopoint_proxy->go("",2000,-2000,0); //upper right corner
	break;
      default:
	gotopoint_proxy->go("",0,0,0); //Center of the room
	break;
    }
  robotState = State::WAIT; //Have to wait until we reached the designated place
}

void SpecificWorker::handlerReleaseState()
{
  cout << "RELEASING BOX"<< endl;
  //After moving the box to the corner
  holdingBox = false;
  robotState = State::IDLE;
}


void SpecificWorker::getMarcas () {
  RoboCompGetAprilTags::listaMarcas lstMarcas;
  try{
    lstMarcas = getapriltags_proxy->checkMarcas();//Getting the actual list of marks that the robot sees
  }catch(const Ice::Exception &e){
    std::cout << "Problema con getapriltags_proxy->checkMarcas();" << endl;
  }
  
  for(auto l:lstMarcas) { //For each mark on the list
    //std::cout << "marca.id ->" << l.id << endl;
    //If its the mark of the box that we are looking for, and we didnt move that box before
    if (l.id == currentBox){
      std::cout << "Found current box, id = " << l.id << ", lx = " << l.tx << ", lz = " << l.tz << endl;
      currentMarca = l;
      if(!holdingBox)
	target = innermodel->transform("world", QVec::vec3(currentMarca.tx, 0, currentMarca.tz), "base"); //Setting the target
      else
	//target = innermodel->transform("world", QVec::vec3(2000, 0, 2000), "base"); //TODO CORNER
	gotopoint_proxy->go("",2000,2000,0);
       std::cout << "-------------------------------------" << endl;
      
      boxDistance = target.norm2(); //Initial box distance
      targetActive = true; //have a target to go to
      break;
    }   
  }
}










