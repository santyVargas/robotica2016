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
  try{
  innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
  }catch(exception){ qDebug()<<"Error en 'setParams'"; }
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
  /*** Implementación del componente InnerModel ***
   ** (código realizado en clase) no borrar **/
  try{
   RoboCompDifferentialRobot::TBaseState bState;
   differentialrobot_proxy->getBaseState(bState);
   
   RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
   innermodel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
   
   float dist = ldata[10].dist;
   
   if(target.isActive()){
     state = 0; // avanzar
     //state = esquivarCajasLaser(state);
     
     switch(state)
     {
       case 0:
	 gotoT(dist); // ir al objetivo
	 break;
       case 1:
	 esquivarCajasLaser(dist); // estadoBug
	 break;
     }
   }
  }catch(exception){ qDebug()<<"Error en 'compute'"; }
  
  /*** Nuestro código 		***
   * (Calculo de TR manal)	***//*
   RoboCompDifferentialRobot::TBaseState bState;
   differentialrobot_proxy->getBaseState(bState);
   
   float avance, B, t_r1, t_r2;
   float tR[2];
   
   t_r1 =(target.getPose()[0]-bState.x);
   //t_r2 =(target.getPose()[1]-bState.z); // para usar con lo de la clase
   t_r2 =(target.getPose()[2]-bState.z); // para usar con nustro código
   
   tR[0]=(cos(bState.alpha)*t_r1) + (-sin(bState.alpha)*t_r2);
   tR[1]=(sin(bState.alpha)*t_r1) + (cos(bState.alpha)*t_r2);
   
   B=atan2(tR[0],tR[1]); // Hay algo mal con el calculo del ángulo
   avance=sqrt(tR[0]*tR[0]+tR[1]*tR[1]); 
   
   
   /******** Controlador isActive() ************
    * (No cambiar, funciona con método InnerModel)
    * asi que debe funcionar con nuestro código
    * 						*/
  /* if(target.isActive())
  {
    qDebug()<<"Agulo: "<<B<<", Distancia: "<<avance;
   if(avance < 30 )//lega a destino
   {
     qDebug()<<"destino alcanzado";
     differentialrobot_proxy->setSpeedBase(0,0);
     target.setActive(false);
   }else if(fabs(B) > 0.05)
   {
     differentialrobot_proxy->setSpeedBase(0, B*2);// gira 
   }else
     differentialrobot_proxy->setSpeedBase(avance*1.5, B); //avanza
   
  }  */
  
}


void SpecificWorker::esquivarCajasLaser(float dist){
  //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 
  
   try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.
	
	  dist = ldata[5].dist;
	  float vrot, vadvance;
	  vrot = 1/1000*dist-0.5;
	  if(ldata[5].angle < 0)
	{
	  qDebug()<<"izquierda";
	  differentialrobot_proxy->setSpeedBase(10, vrot); // gira a la izquierda
	}  
	else{
	  qDebug()<<"izquierda";
	  differentialrobot_proxy->setSpeedBase(10, vrot); // sgira derecha
	}
	  
	  
    
    }catch(exception){}
  

  
  
  //qDebug()<<"Entra en esquivar";
	
  /*int state;
  try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[10].dist < 400) 
    {
        std::cout << ldata.front().dist << std::endl;
	float gain = (float)qrand()/RAND_MAX;
	
	if(ldata[5].angle < 0)
	{
	  state = 1; // entra en BugState
	  differentialrobot_proxy->setSpeedBase(10, rot*gain); // gira a la izquierda
	}  
	else{
	  state = 1; // entra en BugState
	  differentialrobot_proxy->setSpeedBase(10, -rot*gain); // sgira derecha
	}
	
        usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec, espera antes de girar
    }
    else
    {
        differentialrobot_proxy->setSpeedBase(200, 0); 
    }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }//fin try
  */
}


void SpecificWorker::setPick(const Pick &myPick)
{
  qDebug()<<"usando myPick: x = "<<myPick.x<<", z = "<<myPick.z;
  target.copy(myPick.x, myPick.z);
  target.setActive(true); // se activa Target
}

void SpecificWorker::gotoT(float dist) // método usado en complemento con InnerModel, No cambiar
{
  QVec tr = innermodel->transform("base",target.getPose(),"world"); // uso del método transform tomando la posición del Target
  float c=atan2(tr.x(),tr.z()); // calculo del angulo
  float d=tr.norm2(); // calculo de la distancia
  
  qDebug()<<"Agulo: "<<c<<", Distancia: "<<d;
  
  //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
  //float dist = ldata[10].dist;
  
  if(dist<400){
    state = 1;
    qDebug()<<"Entra caja";
  }
  
  if(d < 30 ) // sí distancia es menor a 30, llega al destino
   {
     qDebug()<<"destino alcanzado";
     differentialrobot_proxy->setSpeedBase(0,0);
     target.setActive(false); // se desactiva Target
   }else if(fabs(c) > 0.05)
   {
     differentialrobot_proxy->setSpeedBase(0, c*2); // girar (si no te gusta que sea tan rápido, disminuye el segundo parámetro).
   }else
     differentialrobot_proxy->setSpeedBase(d*1.5, c); //avanzar (bajar velocidad, primer parámetro)
}

void SpecificWorker::bug()
{
  /*
  if(cruzarLinea)
  {
    state = State::gotoT();
    return;
  }
  
  float d = laser[10].dist;
  if(d>160){ // gira derecha
    vr = -0.2;
  }
  if(d<130{
    vr = 0.2;
  }
  //float vadv = ex
  */
}
