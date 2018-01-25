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
	void newAprilTag(const tagsList &tags);
	tag nearestAprilTag(const tagsList &tags);
	tag testAprilTag(const tagsList &tags);
public slots:
	void compute(); 	
    
private:
	bool searching = false; //false = initial box founded
	int initialBox = 11;
	int currentBox = 11;
	int dropPlace = 0;
	InnerModel *innermodel;	
	tag currentTag;
	QVec target;
	bool targetActive = false;
	
};

#endif

