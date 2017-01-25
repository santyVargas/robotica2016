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
  //state = State::IDLE;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
  try
  {
    state=State::INIT;
    innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
  }catch(exception)
  { 
    qDebug()<<"Error en 'setParams'"; 
    
  }
    timer.start(Period);
    return true;
}

void SpecificWorker::compute()
{
  /*** Implementación del componente InnerModel ***
   ** (código realizado en clase) no borrar **/
  
  try
  {
   differentialrobot_proxy->getBaseState(bState);

   innermodel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
   
   float dist = ldata[10].dist;
   QVec ini;
   
   if(target.isActive())
   {
     
     switch(state)
     {
       case State::INIT:
	 if ( target.isActive() )
	 {
	   qDebug() << "DE INIT a GOTO";
	   state = State::GOTO;
	 }
	 break;
       case State::GOTO:
	 checkAngle=true;
	 gotoTarget(dist);
	 break;
       case State::BUGINIT:
	 bugInit();
	 break;
       case State::BUG:
	 bug();
	 break;
       default:
	 break;
     }
   }
  }catch(exception){
    std::cout << "exepcion" << std::endl;
  }
  
}

void SpecificWorker::gotoTarget(float dist) // método usado en complemento con InnerModel, No cambiar
{
  QVec tr = innermodel->transform("base",target.getPose(),"world"); // uso del método transform tomando la posición del Target
  float c=atan2(tr.x(),tr.z()); // calculo del angulo
  float d=tr.norm2(); // calculo de la distancia
  
  //qDebug()<<"Agulo: "<< c <<", Distancia: "<<d;

  if(d < 350 ) // sí distancia es menor a 30, llega al destino
   {
     qDebug()<<"destino alcanzado";
     differentialrobot_proxy->setSpeedBase(0,0);
     
     //hay que hacer que siga moviendose solo     
     target.setActive(false); // se desactiva Target
     state=State::INIT;
   }else if(fabs(c) > 0.05)
   {
     differentialrobot_proxy->setSpeedBase(0, c*2); 
   }else
   {
     if(d>600)
     {
       differentialrobot_proxy->setSpeedBase(d*0.2, c);
     }else
     {
       differentialrobot_proxy->setSpeedBase(d*1.05, c);
     }  
   }
   
   if(obstacle()==true)
   {
    state=State::BUGINIT;
    return;
  }
}
 
void SpecificWorker::bug()
{ 
  qDebug()<<"BUG";
  
  if(targetAtSight()==true){
    state = State::GOTO;
    qDebug()<<"Objetivo visible";
  }
  
  if(secondDist()==true)
  {
    state=State::BUGINIT;
    return;
  }
  differentialrobot_proxy->setSpeedBase(50, -0.2);
}

void SpecificWorker::bugInit()
{ 
  qDebug()<<"BUG INIT";
  if(targetAtSight()==true){
    state = State::GOTO;
    qDebug()<<"Objetivo visible";
  }
  
  if(checkAngle)
  {
     staticAngle = bState.alpha + 1.5707;
     checkAngle=false;
  }

  //if(bState.alpha<staticAngle) // hasta que obstacle sea falso
  //if(bState.alpha<staticAngle)
  
  if(obstacle()==true)
    differentialrobot_proxy->setSpeedBase(50, 0.6); // gira a la derecha
  else
  {
    //differentialrobot_proxy->setSpeedBase(0, 0);
    state=State::BUG;
  }
}

bool SpecificWorker::obstacle()
{
   try
    {
	ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[12].dist < MAX_ADVANCE){ 
      
      return true;
      
    }else
      return false;
    }catch(exception){}

}

bool SpecificWorker::secondDist()
{
   try
    {
	ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin()+5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b)
	{ 
	  return     a.dist < b.dist; 
	  
	}) ;  //sort laser data from small to large distances using a lambda function.

    if( ldata[12].dist < THRESHOLD){ 
      return true;
    }else
      return false;
    }catch(exception){}
}


bool SpecificWorker::targetAtSight() // hacemos funcionar este método y creo ya quedaria todo
{
  
  QPolygon poly;
	for ( auto l: ldata )
	{
		QVec r = innermodel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		poly << p;
	}
	QVec targetInRobot = innermodel->transform("base", target.getPose(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 200);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(target.getPose().x(),target.getPose().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = innermodel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innermodel->transform("world", point - QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innermodel->transform("world", point + QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poly.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
	return true; 
}

void SpecificWorker::setPick(const Pick &myPick)
{
  qDebug()<<"usando myPick: x = "<<myPick.x<<", z = "<<myPick.z;
  target.copy(myPick.x, myPick.z);
  target.setActive(true); // se activa Target
  state= State::INIT;
}

bool SpecificWorker::atTarget()
{
  return !target.isActive();
}

void SpecificWorker::go(const string& nodo, const float x, const float y, const float alpha)
{
  target.copy(x, y);
  target.setActive(true); // se activa Target
  state= State::INIT;
}

