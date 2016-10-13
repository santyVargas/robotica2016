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
  bool comprueba=false;
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState); 
  
  float xPrima, yPrima, anguloDer, anguloIzq, vadvance, dist, normal;
  //const float threshold = 415; //millimeters, distancia con obstaculos
  
  if(target.isActive())
  {
    comprueba=true;
    //differentialrobot_proxy->setSpeedBase(200, 0); 
   target.setActive(false);
  
   QVec t = target.getPose();
  
   //QVec r = QVec::vec2(bState.x, bState.z);
   //float distNorm = (t-r).norm2(); // distancia entre los dos puntos
   QVec r;
   r.resize(2);
   r[0]= bState.x;
   r[1]= bState.z;
   //float distNorm = (t-r).norm2();
  
  // pasar target al sistema de referencia del Robot   
  // calcular ángulo con atan2

   xPrima = ((cos(bState.alpha) * bState.x) + (-sin(bState.alpha) * bState.z)) + t[0]; //x'
   yPrima = ((sin(bState.alpha) * bState.x) + (cos(bState.alpha) * bState.z)) + t[1]; //z'
   
   anguloDer = atan2(xPrima, yPrima);
   
   //ANGULO NEGATIVO
   xPrima = ((cos(bState.alpha) * bState.x) + (sin(bState.alpha) * bState.z)) + t[0]; //x'
   yPrima = ((-sin(bState.alpha) * bState.x) + (cos(bState.alpha) * bState.z)) + t[1]; //z'
   anguloIzq = atan2(xPrima, yPrima);;
   
    //qDebug()<<"angulo " << angulo;
   
   /*
    Convertimos los radianes a grados
      NumGrados=Radianes*(180/PI);
      Conertimos los grados a radianes
      NumRadianes=Grados*(PI/180); 
    */
  
   
  // calcular vavance a partitr de dist ******************************
    
   dist=sqrt(pow(bState.x-xPrima,2) + pow(bState.z - yPrima,2));
  
   //normal= norm(dist - angulo);
   
   qDebug()<<"angulo " << anguloDer << "robot:" << bState.alpha;
   
    
   /*
   if(bState.alpha < angulo ){
     differentialrobot_proxy->setSpeedBase(5, angulo);
      
   }else 
     if(bState.alpha > angulo ){
     differentialrobot_proxy->setSpeedBase(5, -angulo);
     }else{
	  differentialrobot_proxy->setSpeedBase(0, 0);
      }
      */
      
   
   //vadvance = dist;
      
  // if (vadvance > MAX_ADVANCE)
    // vadvance = MAX_ADVANCE;
  
  // calcular vrot a partit del angulo

  }// final if target
  
  if(comprueba){
     try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.

      if( dist < MAX_ADVANCE) //
      {
        std::cout << ldata.front().dist << std::endl;
	
	//float gain = (float)qrand()/RAND_MAX;
	
	if(bState.alpha < anguloDer){
	  differentialrobot_proxy->setSpeedBase(10, anguloDer); // gira a la izquierda 
      }  
	else
	  differentialrobot_proxy->setSpeedBase(10, -anguloDer); // sgira derecha
	  
	
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
  }
  
/*  
  
  float rot = 0.6;  //rads per second

    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[5].dist < threshold) //
    {
        std::cout << ldata.front().dist << std::endl;
	
	float gain = (float)qrand()/RAND_MAX;
	
	if(ldata[5].angle < 0){
	  differentialrobot_proxy->setSpeedBase(10, rot*gain); // gira a la izquierda
    }  
	else
	  differentialrobot_proxy->setSpeedBase(10, -rot*gain); // sgira derecha
	  
	
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

void SpecificWorker::setPick(const Pick &myPick){
  
  //float rot = 0.6;

  qDebug()<<"usando myPick: x = "<<myPick.x<<", y = "<<myPick.y<<", z = "<<myPick.z;
  target.copy(myPick.x, myPick.z);
  target.setActive(true);
  
} 
