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

/**
       \brief
       @author Santiago Vargas M.
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);


public slots:
	void compute(); 	

private:
  enum class State{SEARCH, WAIT};
  
  InnerModel *innerModel;
  int current = 0;
  
  struct Tag
  {
    InnerModel *inner;
    void init(InnerModel *inner_)
    {
      inner = inner_;    
    }
    bool active=false;
    mutable QMutex m;
    QVec pose;
    int id;
  
    void setActive(bool V)
    {
      QMutexLocker ml(&m);
      active=V;
    }
    void copy(float x, float z, int id_)
    {
      QMutexLocker ml(&m); 
      pose.resize(3);
      QVec r = inner->transform("world", QVec::vec3(x,0,z), "rgbd");
      pose[0]=r.x();
      pose[1]=0;
      pose[2]=r.z();
      id = id_;
  }
  
  bool isActive()
  {
    QMutexLocker ml(&m); 
    return  active;
  }
  QVec getPose()
  {
    QMutexLocker ml(&m); 
    return  pose;
  } 
  QVec getId()
  {
    QMutexLocker ml(&m); 
    return  id;
  }   
  void print()
  {
    qDebug() << "Tag" << id;
    pose.print("pose");
  }
  
  
 };
  
  Tag tag;
  State state= State::SEARCH;
	
};

#endif

