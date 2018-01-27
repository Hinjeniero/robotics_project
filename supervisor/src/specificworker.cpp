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
  differentialrobot_proxy->getBaseState(state);
  
  getMarcas();
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
    std::cout << "SEARCH - Buscando la id " << currentBox <<endl;
    if(targetActive){  
      
      std::cout << "SEARCH - De camino a target X = "<< target.x() << " Z = " << target.z() <<endl;
      std::cout << "SEARCH - De camino a auxTarget X = "<< auxTarget.x() << " Z = " << auxTarget.z() <<endl;
      //target = innermodel->transform("world", QVec::vec3(target.x(), 0, target.z()), "base");
      
      gotopoint_proxy->go("", target.x(), target.z(), 0);      
      robotState = State::WAIT;
    }
}

void SpecificWorker::waitState(){
  std::cout << "WAIT" << endl;
  if (gotopoint_proxy->atTarget()){
    std::cout << "arrived at target wait" << endl;
    gotopoint_proxy->stop();
    //TODO Aquí ha llegado a la caja
    //gotopoint_proxy->go("", 1700, 1900, 0);
    
    //robotState = State::IDLE;
  }
}

void SpecificWorker::idleState(){
  std::cout << "IDLE" << endl;
  if (gotopoint_proxy->atTarget()){
    std::cout << "atTarget()" << endl;
    //current++;
    //if (current > 13)
    //  current = 10; //TODO delete this later :) 
    gotopoint_proxy->stop();
    targetActive = false;
    //robotState = State::SEARCH;
  }
}

void SpecificWorker::getMarcas () {
  try{
    lstMarcas = getapriltags_proxy->checkMarcas();
  }catch(const Ice::Exception &e){
      std::cout << "Problema con getapriltags_proxy->checkMarcas();" << endl;
  }
  for(auto l:lstMarcas) {
    std::cout << "marca.id ->" << l.id << endl;
    if(!searching) {					// initialBox not founded yet
	std::cout << "!searching" << endl;
	if (l.id == initialBox){			// searching for initialBox
	  std::cout << "Encontrada la caja inicial con l.id = " << l.id << " lx = " << l.tx << " lz = " << l.tz << endl;
	  auxTarget = innermodel->transform("world",QVec::vec3(l.tx, 0, l.tz) , "base");
	  auxTag.id = l.id;
	  auxTag.tx = l.tx;
	  auxTag.tz = l.tz;	  
	  //std::cout << "Encontrada la caja inicial con auxTarget.x = " << auxTarget.x() << " auxTarget.z() = " << auxTarget.z() << endl;
	  target = innermodel->transform("world", QVec::vec3(auxTag.getX(), 0, auxTag.getZ()) , "base");
	  std::cout << "Encontrada la caja inicial con target.x = " << target.x() << " target.z() = " << target.z() << endl;
	  
	  boxDistance = target.norm2();
	  targetActive = true;
	}
    } else {						// initialBox founded
	std::cout << "searching" << endl;
	if (l.id == currentBox){			// searching for currentBox
	  //currentTag = l;
	  std::cout << "Encontrada la segunda caja" << endl;
	  targetActive = true;
	}
    }     
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


/*void SpecificWorker::newAprilTag(const tagsList &tags){
  std::cout << "newAprilTag()" << endl;
  std::cout << "currentTag.id = " << currentTag.id << endl;
  testAprilTag(tags);*/
  /*if(!searching) {					// initialBox not founded yet
    std::cout << "!searching" << endl;
    if (currentTag.id == initialBox)			// searching for initialBox
      currentTag = testAprilTag(tags);
  } else {						// initialBox founded
    std::cout << "searching" << endl;
    if (currentTag.id == currentBox)			// searching for currentBox
    currentTag = testAprilTag(tags);
  }  */
  
  /*if (robotState == State::SEARCH && currentTag.id == currentBox) {
    std::cout << "Encontrada! Current - "<< currentBox << " -> tag id ->" << currentTag.id <<endl;
    std::cout << "Antes de transformalar las coordenadas X = "<< currentTag.tx << " Z = " << currentTag.tz << endl;
    target = innermodel->transform("world", QVec::vec3(currentTag.tx, 0, currentTag.tz), "rgbd");
    std::cout << "World - rgbd -> X = "<< target.x() << " Z = " << target.z() << endl;
    
    target = innermodel->transform("world", QVec::vec3(currentTag.tx, 0, currentTag.tz), "base");
    std::cout << "World - Base -> X = "<< target.x() << " Z = " << target.z() << endl;
    
    target = innermodel->transform("base", QVec::vec3(currentTag.tx, 0, currentTag.tz), "world");
    std::cout << "base - world -> X = "<< target.x() << " Z = " << target.z() << endl;
    
    target = innermodel->transform("base", QVec::vec3(currentTag.tx, 0, currentTag.tz), "rgbd");
    std::cout << "Base - rgbd -> X = "<< target.x() << " Z = " << target.z() << endl;
    
    target = innermodel->transform("rgbd", QVec::vec3(currentTag.tx, 0, currentTag.tz), "base");
    std::cout << "rgbd - Base -> X = "<< target.x() << " Z = " << target.z() << endl;
    
    target = innermodel->transform("rgbd", QVec::vec3(currentTag.tx, 0, currentTag.tz), "world");
    std::cout << "rgbd - world -> X = "<< target.x() << " Z = " << target.z() << endl;
    targetActive = true;
  }
  std::cout << currentTag.id << endl;
  std::cout << "X = "<< target.x() << " Z = " << target.z() << endl;
  std::cout << "*****************************************" << endl;
}

void SpecificWorker::testAprilTag(const tagsList &tags) {
  //choose only the box with the currentBox's id on it
  std::cout << "TESTAPRILTAGS" << endl;
  for (auto t: tags) {
    if(!searching) {
      if (t.id == initialBox) {
	currentTag = t;
	std::cout << "testAprilTag()- !searching devolvera -> X = "<< currentTag.tx << " Z = " << currentTag.tz << endl;
      }
    } else {
      if (t.id == currentBox) {
	currentTag = t;
	std::cout << "testAprilTag() searching devolvera -> X = "<< currentTag.tx << " Z = " << currentTag.tz << endl;
      }
    }
  }
}

//tag SpecificWorker::nearestAprilTag(const tagsList &tags){
/*  //sin periferia, la mas cercana de las que vea
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
 return tmin;*/
//}*/

void SpecificWorker::handlerState() {
  cout << "REALIZANDO TRABAJO MANUAL "<< endl;
  robotState = State::IDLE;
}



