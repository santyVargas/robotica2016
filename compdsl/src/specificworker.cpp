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
 
  if(target.isActive())
  {
    
      float c;
      float d;
      getDisAndRot(c,d);

      
      float kr=0.5;
      float kv=0.1;
      float ec=0.2;
      float ed=1;
      
      printf("c= %f d= %f\n",c,d);
      
      if(abs(c)>ec )
      differentialrobot_proxy->setSpeedBase(0,kr* c);
      else if(d>ed)
	differentialrobot_proxy->setSpeedBase(kv*d,0);
      else{
	target.setActive(false);
	printf("destino alcanzado");
      }
    }
}


void SpecificWorker::getDisAndRot(float &c,float &d){
      RoboCompDifferentialRobot::TBaseState bState;
      differentialrobot_proxy->getBaseState( bState);
      //Taux= vector que va desde el robot al target en cordenadas globales
      float Taux[2];
      Taux[0]=(target.getPose()[0]-bState.x);
      Taux[1]=(target.getPose()[1]-bState.z);
      
      //MatRot matriz de rotacion para girar el vector Taux en sentido contrario
      //a la rotacion alpha del robot
      float MatRot[2][2];
      MatRot[0][0]=cos(bState.alpha);MatRot[0][1]=-sin(bState.alpha);
      MatRot[1][0]=sin(bState.alpha);MatRot[1][1]=cos(bState.alpha);
      
      //Tr contiene las cordenadas de target respecto al sistema de cordenadas
      //del robot
      float Tr[2];
      Tr[0]=MatRot[0][0]*Taux[0]+MatRot[0][1]*Taux[1];
      Tr[1]=MatRot[1][0]*Taux[0]+MatRot[1][1]*Taux[1];
      
      c=qAtan2(Tr[0],Tr[1]);
      d=sqrt(Tr[0]*Tr[0]+Tr[1]*Tr[1]);
  
}


void SpecificWorker::setPick(const Pick &myPick){
  
  //float rot = 0.6;

  qDebug()<<"usando myPick: x = "<<myPick.x<<", y = "<<myPick.y<<", z = "<<myPick.z;
  target.copy(myPick.x, myPick.z);
  target.setActive(true);
  
} 
