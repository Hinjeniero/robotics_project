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
	void searchState(RoboCompAprilTags::tagsLis tag);
	void waitState();
	/*State machine enum*/
	enum State {SEARCH, WAIT};
      /*Present state of the robot*/
	State robotState = State::SEARCH;
	void newAprilTag(const tagsList &tags);
	

public slots:
	void compute(); 	
    
private:
	int current=0;
	struct tagBuffer
	{
	  tagsList buf;
	  QMutex mutex;
	  void put(tag t)
	  {
	    QMutexLocker ml(&mutex);
	    buf.insert(0, t);
	  };
	  tag get()
	  {
	    QMutexLocker ml(&mutex);
	    tag t = buf.back();
	    buf.pop_back();
	    return t;
	  };
	};tagBuffer buffer;
  
};

#endif

