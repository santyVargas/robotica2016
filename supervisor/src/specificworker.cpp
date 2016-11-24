/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
	innerModel= new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	tag.init(innerModel);
	
	timer.start(Period);	
	return true;
}

void SpecificWorker::compute()
{
  try
  {
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
  }
  catch(const Ice::Exception &e)
  {
	std::cout << "Error reading from Camera" << e << std::endl;
	return;
  }
 
   switch(state)
   {
     case State::SEARCH:
  
	  if( tag.getId() == current)
	  {
	    differentialrobot_proxy->stopBase();
	    gotopoint_proxy->go(" ", tag.getPose().x(), tag.getPose().z(), 0);
	    qDebug() << "   IMAGEN   VISTA:  "<<current;
	    qDebug() << "Me llego x: "<<  tag.getPose().x()<<", z: "<< tag.getPose().z();
	    state = State::WAIT;
	  }
	  else{
	     differentialrobot_proxy->setSpeedBase(0, .3);
	  }
	 
	  break;

      case State::WAIT:
	//differentialrobot_proxy->setSpeedBase(0, 0);
	//qDebug() << "estoy en WAIT";
	  if (gotopoint_proxy->atTarget() == true )
	  {
	    
	    qDebug()<<"      LLEGO";
	    differentialrobot_proxy->stopBase();
	    current++%4;
	    state = State::SEARCH;
	  }
	  
	   break;
	  

   }
}

///////////////////////////////////
//SUBSCRIPTION
///////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList &tags)
{
  //qDebug() << "Me llego "<< tags[0].id << tags[0].tx << tags[0].tz;
   
  tag.copy(tags[0].tx, tags[0].tz, tags[0].id ); 
  //tag.print();
}


