/*
void SpecificWorker::printState(float d, float adv, float rot){
  std::cout << "-------------------------"<< endl;
  std::cout << "Sigmoid - " << getSigmoid(d) << endl;
  std::cout << "Gauss - " << getGauss(rot, 0.3, 0.5) << endl;
  std::cout << "Vel is - " << adv << endl;
  std::cout << "Rotation is - " << rot << endl;
  std::cout << "Distance is - " << d << endl;
  std::cout << "-------------------------"<< endl;
}

//y = x1-x2 | x = y2-y1 | b = (x*-x1)+(y*-y1)
//Substitute point in mx + ny + b, and if it equals 0, it is in the slope
bool SpecificWorker::vectorContainsPoint (std::pair <float, float> point) {
  std::pair <float, float> end = target.getTarget();
  std::pair <float, float> start = target.getRobotPos();
  
  float x = end.second-start.second;
  float y = start.first - end.first;
  float b = (-x*start.first)+(-y*start.second);
  return x*point.first + y*point.second + b < marginError; //the point is in the neighborhood of the vector.
}

void SpecificWorker::turnState (RoboCompLaser::TLaserData ldata) {
  std::cout << "TURN STATE!" << endl;  
  if(endTurnState(ldata)){ //Obstacle sorting, checks if there is still an obstacle
    differentialrobot_proxy->setSpeedBase(0, 0);
    robotState = State::AVOID;
    return;
  }
  if(turnDirection == Turn::LEFT){
    differentialrobot_proxy->setSpeedBase(0, -1);
    return;
  }
  differentialrobot_proxy->setSpeedBase(0, 1); 
}

//Checks it there is an obstacle after turning, taking into account the direction of the last turn
bool SpecificWorker::endTurnState(RoboCompLaser::TLaserData ldata){ 
  int start = middleAngle;
  int end = rightAngle;
  if(turnDirection == Turn::RIGHT){  
    start = leftAngle;
    end = middleAngle; 
  }
  for (int i = start; i<= end; i++){
    if(ldata[i].dist < threshold)
      return false;
  }
  return true;
}

RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort(ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	differentialrobot_proxy->setSpeedBase(400, 0);
	if(ldata[20].dist < threshold)
	{
		//std::cout << ldata.front().dist << std::endl;
 		//differentialrobot_proxy->setSpeedBase(10, rot);
		differentialrobot_proxy->setSpeedBase(0, 1);
		usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
	}

	try
	{
		camera_proxy->getYImage(0,img, cState, bState);
		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
		searchTags(image_gray);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}

*/
