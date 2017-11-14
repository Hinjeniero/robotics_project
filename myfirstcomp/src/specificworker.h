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

#define MAXVEL 400 //Max advancing speed
#define MAXROT 1 //Max rotation speed
#define MINDISTANCE 50 //Distance to the target needed to arrive at it.
#define threshold 300 //Minimal distance to an obstacle before action
#define leftMaxAngle 10 //First valid position of the laser
#define leftAngle 25 //25º position of the index
#define middleAngle 50
#define rightAngle 75
#define rightMaxAngle 90 //Last valid position of the laser
#define marginError 100 //Margin error of the vectorContainsPoint
#define angleLimit 0.06//Angle which the robot has with the target. threshold

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	/*Constant containing the euler constant (e number)*/
	const float EulerConstant = std::exp(1.0);
	/*Set the initial params of the world, including the world's path*/
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	/*Working method of the idle State (Only waiting for a target)*/
	void idleState();
	/*GOTO State -> Going to a target*/
	void gotoState(RoboCompLaser::TLaserData ldata); 
	/*TURN State -> Turning until the encountered obstacle is no more at sight*/
	void turnState (RoboCompLaser::TLaserData ldata);
	/*Checks the laser and returns TRUE when the turn is enough to not bump into the obstacle*/
	bool endTurnState(RoboCompLaser::TLaserData ldata);
	/*AVOID State -> Follow the obstacle's border until it sees the target or pass over the vector of GOTO*/
	void avoidState (RoboCompLaser::TLaserData ldata); 
	/*Returns TRUE if the target is within the robot's laser (ldata). FALSE otherwise*/
	bool targetAtSight(RoboCompLaser::TLaserData ldata);
	/*Returns TRUE if the robot's present position is near (Using a margin error) of the GOTO vector*/
	bool vectorContainsPoint (std::pair <float, float> point);
	/*Return True if the target is almost in front of the robot (low angle) (substitute of line's equation)*/
	bool angleWithTarget ();
	/*END State -> Last state between GOTO and IDLE. Does nothing atm*/
	void endState ();
	/*Returns TRUE if there is an obstacle in the laser data at a distance lower than a threshold*/
	bool obstacle(RoboCompLaser::TLaserData ldata);
	/*Decides the turn direction, thinking in which one it would have to turn the less*/
	void decideTurnDirection(RoboCompLaser::TLaserData ldata);
	/*Print parameters of the GOTO state->Sigmoid value, gaussian value, rot and adv...*/
	void printState(float d, float adv, float rot);
	/*Print the laser's array values between start and end positions*/
	void printLaser(RoboCompLaser::TLaserData ldata, int start, int end);
	/*Set a target according to the pick that the user did with the mouse*/
	void setPick(const RoboCompRCISMousePicker::Pick& pick);
	/*Gauss function*/
	float getGauss(float Vr, float Vx, float h);
	/*Sigmoid function*/
	float getSigmoid(float distance);
	/*State machine enum*/
	enum State {IDLE, GOTO, TURN, AVOID, END};
	/*Turn direction enum*/
	enum Turn {NONE, LEFT, RIGHT};
	/*Present state of the robot*/
	State robotState = State::IDLE;
	/*Last turn direction that the robot did*/
	Turn turnDirection = Turn::NONE;
	/*Where was the last wall (To wall follow, bug method)*/
	Turn lastWall = Turn::NONE;
	
	/*Where was the last wall (To wall follow, bug method)*/
	void go(const string &nodo, const float x, const float y, const float alpha);
	void turn(const float speed);
	bool atTarget();
	void stop();
	

public slots:
	void compute(); 	

private:
	InnerModel *inner;
	struct Target
	{
	    bool empty = true;
	    float x;
	    float z;
	    float robotX;
	    float robotZ;
	    QMutex mutex;
	    bool isEmpty()
	    {
	      QMutexLocker ml(&mutex);
	      return empty;
	    } 
	    
	    void setEmpty()
	    {
	      QMutexLocker ml(&mutex);
	      empty = true;
	    };
	    void setTarget(float x_, float z_)
	    {
	      QMutexLocker ml(&mutex);
	      empty = false;
	      x = x_;
	      z = z_;
	    };	    
	    std::pair<float, float> getTarget()
	    {
	      QMutexLocker ml(&mutex);
	      return std::make_pair(x,z);
	    };
	    void setRobotPos(float robot_x, float robot_z)
	    {
	      QMutexLocker ml(&mutex);
	      robotX = robot_x;
	      robotZ = robot_z;
	    };	
	    std::pair<float, float> getRobotPos()
	    {
	      QMutexLocker ml(&mutex);
	      return std::make_pair(robotX,robotZ);
	    };
	};
	Target target;
};

#endif

