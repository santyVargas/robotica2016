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
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
   RoboCompDifferentialRobot::TBaseState bState;
   differentialrobot_proxy->getBaseState(bState);
      
   float angulo; //angulo
   float distancia; //distancia
   float meta; //meta
    
   float vTarget[2];
      
         
   vTarget[0]=(cos(bState.alpha)*(target.getPose()[0]-bState.x)) + (-sin(bState.alpha)*(target.getPose()[1]-bState.z));
   vTarget[1]=(sin(bState.alpha)*(target.getPose()[0]-bState.x)) - (cos(bState.alpha)*(target.getPose()[1]-bState.z));
      
   angulo=atan2(vTarget[0],vTarget[1]);
     
   meta=sqrt(vTarget[0]*vTarget[0]+vTarget[1]*vTarget[1]);
   distancia=sqrt( ((bState.x-vTarget[0])*(bState.x-vTarget[0])) + ((bState.z-vTarget[1])*(bState.z-vTarget[1])) );
    
    
  if(target.isActive())
  {    
    
    qDebug()<<"angulo= " << angulo <<" distancia= " << distancia << "meta= " << meta;
    
    if(meta<5)
      {
	qDebug()<<"destino alcanzado";
	differentialrobot_proxy->setSpeedBase(0,0);
	target.setActive(false);
      }
      
    else{
    
      if(abs(angulo)>0.2)
	differentialrobot_proxy->setSpeedBase(0, angulo * 0.5); //gira
      else
      {
	if(distancia>1)
	  differentialrobot_proxy->setSpeedBase(0.1*distancia+50,0); // avanza
      }
    }
  }
}


void SpecificWorker::setPick(const Pick &myPick){
  
  //float rot = 0.6;

  qDebug()<<"usando myPick: x = "<<myPick.x<<", y = "<<myPick.y<<", z = "<<myPick.z;
  target.copy(myPick.x, myPick.z);
  target.setActive(true);
  
} 
