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
    innermodel = new InnerModel("/home/hinjeniero/robocomp/files/innermodel/betaWorldArm.xml");
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
  innermodel->updateTransformValues("base", state.x, 0, state.z, 0, state.alpha, 0); 
  
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
	case State::HANDLER:
	  try
	  {	
	    handlerState();
	  }catch(const Ice::Exception &ex) {
	    std::cout << "Exception in handlerState - "<< ex <<endl; 
	  }
	  break;
      }
  }catch(const Ice::Exception &ex) {
    std::cout << "Exception in switch robotState supervisor - "<< ex <<endl; 
  }
}

void SpecificWorker::searchState(){
    gotopoint_proxy->turn(1);
    std::cout << "SEARCH - Buscando la id " << currentBox <<endl;
    getMarcas();
    if(targetActive){  
      std::cout << "SEARCH - De camino a target X = "<< target.x() << " Z = " << target.z() <<endl;
      gotopoint_proxy->go("", target.x(), target.z(), 0);      
      robotState = State::WAIT;
    }
}

void SpecificWorker::waitState(){
  std::cout << "WAIT state" << endl;
  if (gotopoint_proxy->atTarget()){
    gotopoint_proxy->stop();
    boxesHistory.addBox(currentMarca);
    gotopoint_proxy->go("",0,0,0);
    robotState = State::IDLE;
  }
}

void SpecificWorker::idleState(){
  std::cout << "IDLE state" << endl;
  if (gotopoint_proxy->atTarget()){
    std::cout << "Idle state, back to the beginning" << endl;
    gotopoint_proxy->stop();
    targetActive = false;
    currentBox++;
    robotState = State::SEARCH;
  }
}

void SpecificWorker::handlerState() {
  cout << "REALIZANDO TRABAJO MANUAL "<< endl;
  //TODO switch case to go to whatever corner is designated after picking the box
  robotState = State::IDLE;
}

void SpecificWorker::getMarcas () {
  try{
    lstMarcas = getapriltags_proxy->checkMarcas();
  }catch(const Ice::Exception &e){
    std::cout << "Problema con getapriltags_proxy->checkMarcas();" << endl;
  }
  
  for(auto l:lstMarcas) {
    std::cout << "marca.id ->" << l.id << endl;
    if (l.id == currentBox && !boxesHistory.containsBox(l)){
      std::cout << "Found current box, id = " << l.id << ", lx = " << l.tx << ", lz = " << l.tz << endl;
      printAllTransformations(l);
      
      std::cout << "After transforming the found box, target.x() = " << target.x() << ", target.z() = " << target.z() << endl;
      currentMarca = l;
      boxDistance = target.norm2(); //Initial box distance
      targetActive = true;
      break;
    }   
  }
}

void SpecificWorker::printAllTransformations(const ::RoboCompGetAprilTags::marca t){
	  std::cout << "NORMAL POSITION |" <<t.tx<<","<<t.tz<<"|"<<endl;
	  QVec dst_1 = innermodel->transform("world", QVec::vec3(t.tx, 0, t.tz), "base");
	  std::cout << "FROM BASE TO WORLD |" <<dst_1.x()<<","<<dst_1.z()<<"|"<<endl;
	  QVec dst_2 = innermodel->transform("world", QVec::vec3(t.tx, 0, t.tz), "rgbd");
	  std::cout << "FROM RGBD TO WORLD |" <<dst_2.x()<<","<<dst_2.z()<<"|"<<endl;
	  QVec dst_3 = innermodel->transform("base", QVec::vec3(t.tx, 0, t.tz), "world");
	  std::cout << "FROM WORLD TO BASE |" <<dst_3.x()<<","<<dst_3.z()<<"|"<<endl;
	  QVec dst_4 = innermodel->transform("base", QVec::vec3(t.tx, 0, t.tz), "rgbd");
	  std::cout << "FROM RGBD TO BASE |" <<dst_4.x()<<","<<dst_4.z()<<"|"<<endl;
	  QVec dst_5 = innermodel->transform("rgbd", QVec::vec3(t.tx, 0, t.tz), "world");
	  std::cout << "FROM WORLD TO RGBD |" <<dst_5.x()<<","<<dst_5.z()<<"|"<<endl;
	  QVec dst_6 = innermodel->transform("rgbd", QVec::vec3(t.tx, 0, t.tz), "base");
	  std::cout << "FROM BASE TO RGBD |" <<dst_6.x()<<","<<dst_6.z()<<"|"<<endl;
}

/*
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
//}*/









