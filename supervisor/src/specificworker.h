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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#define MAXROT 1 //Max rotation speed

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
	void searchState();
	void waitState();
	void idleState();
	void handlerState();
	/*State machine enum*/
	enum State {SEARCH, WAIT, IDLE, HANDLER};
      /*Present state of the robot*/
	State robotState = State::SEARCH;
	RoboCompGetAprilTags::listaMarcas lstMarcas;
	void getMarcas();
	void printAllTransformations(const ::RoboCompGetAprilTags::marca t);
	void newAprilTag(RoboCompGetAprilTags::listaMarcas& tags);
	
	
	
public slots:
	void compute();
    
private:
  
  struct Tag{
    Tag(){};
    
    QMutex mutex;
    float tx,tz;
    int id;    
        
    void setTag(int _id, float _x, float _z) {
      QMutexLocker ml(&mutex);
      id = _id;
      tx = _x;
      tz = _z;      
    }
    
    int getID() {
      QMutexLocker ml(&mutex);
      return id;
    }
    
    int getX() {
      QMutexLocker ml(&mutex);
      return tx;
    }

    int getZ() {
      QMutexLocker ml(&mutex);
      return tz;
    }    
  };
  
  //Para saber que cajas hemos movido
  struct BoxesList{
    std::vector<RoboCompGetAprilTags::marca> movedBoxes;
    QMutex mut;
    
    void addBox(marca m){
      QMutexLocker ml(&mut);
      movedBoxes.push_back(m);
    }    
    bool containsBox(marca m){
      QMutexLocker ml(&mut);
      for (auto mark:movedBoxes){
	 if (m.id == mark.id)
	   return true;
      }
      return false;
    }
    void deleteAllBoxes(){
      QMutexLocker ml(&mut);
      movedBoxes.clear();
    }
    bool isEmpty(int index){
      QMutexLocker ml(&mut);
      return movedBoxes.empty();
    }
    int size(){
      QMutexLocker ml(&mut);
      return movedBoxes.size();
    }
  };
  
  int initialBox = 11;
  int currentBox = initialBox;
  int dropPlace = 0; //0, 1, 2, 3, depends in which corner do we want to be the dumpster;
  InnerModel *innermodel;	
  Tag currentTag;
  RoboCompGetAprilTags::marca currentMarca;
  QVec target;
  float boxDistance;
  bool targetActive = false;
  BoxesList boxesHistory;
  Tag tag;

};
#endif

