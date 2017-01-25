#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time, networkx as nx
import matplotlib.pyplot as plt

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

g = nx.Graph()
preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"AprilTags.ice")
from RoboCompAprilTags import *
Ice.loadSlice(preStr+"GotoPoint.ice")
from RoboCompGotoPoint import *
Ice.loadSlice(preStr+"DifferentialRobot.ice")
from RoboCompDifferentialRobot import *

global estado

class SpecificWorker(GenericWorker):
	rutas=(71) #final del pasillo
	
	listaTargets=['70','61','70']
	
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		
		self.switch={"INIT" : self.init, "TI" : self.ti,
		      "PI" : self.pi,
		      "GOI" : self.goi
		      }
		self.estado="INIT"
		self.grafo()
		#self.bState = TBaseState()
	
	def grafo(self):   
        #creacion del grafo
        #g=nx.Graph()
            self.posiciones = {}
            with open ('src/puntos.txt',"r") as f: #"puntos.txt"
                for line in f:
                    l=line.split()
                    if l[0]=="N":
                       g.add_node(l[1], x= float(l[2]),y=float(l[3]),tipo=l[4])
                       self.posiciones[l[1]] = (float(l[2]),float(l[3]))
                    else:
                       g.add_edge(l[1], l[2])

            #print posiciones
            img = plt.imread("plano.png")
            plt.imshow(img, extent = [-12284,25600,-3840,9023])
            nx.draw_networkx(g, self.posiciones)
 
            #print "Haciendo camino minimo"
            #print nx.shortest_path(g,source="1", target="6")
       
            plt.show()
                   

	def nodoCercano(self):
	  bState = self.differentialrobot_proxy.getBaseState()
	  r = (bState.x , bState.z)
	  dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
	  #funcion que devuele el nodo mas cercano al robot
	  return  sorted(list (( n[0] ,dist(n[1],r)) for n in self.posiciones.items() ), key=lambda s: s[1])[0][0]
	

	#def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		#with open("puntos.txt", "r") as f:
		  #for line in f:
		    #if line[0] == "N" line[1]
		    #g.add_Node(line[1], x=line[2], y=line[3], name="robolab")
		    #g.add.Edge( , )
	#      with open("src/puntos.txt", "r") as f:
	#	for line in f:
	#	  l=line.split()
	#	  if l[0] == "N":
	#	    g.add_node(l[1], x=l[2], y=l[3], tipo=l[4])
	#	  if l[0]=="E":
	#	    g.add_edge(l[1], l[2])
	#     print g    
	      
	#      return True
	      
	 #def readGraph():
	  # with open("puntos.txt", "r") as f:
	#	  for line in f:
	#	    l=line.split()
	#	    if l[0] == "N":
	#	      g.add_Node(l[1], x=l[2], y=l[3], tipo=l[4])
	#	    if l[0]=="E":
	#	      g.add_Ege(l[1], l[2])
	      
	@QtCore.Slot()

	def compute(self):
		
		self.switch[self.estado]()
		
		#if tag #init
		
		#else
		#    if lenght(rutas) == 0 #Ti
			#estado="INIT"
			#self.target=ruta.pop()
			#self.robot=nodoCercano()
			#self.lista=new startedPeriod(start,robot,  )
			#s.estado="Pi"
			
		#    else
		#	if lista #Pi
			    #estado="TI"
			    # controller( lista.pop())
			    # s.estado="GOI"
			    
			
			
		#	else #GOI
		#	    if estado=="GOI"
				#atTarget
				#wait
		
		return True
	    
	def init(self):
	    if len(SpecificWorker.listaTargets)!=0:
		self.estado='TI'
		print 'Lista de nodos a visitar creada'
		print SpecificWorker.listaTargets
	    else :
		print 'Lista vacia'
		quit()
	  
	    print 'En init'
	  
	    return 

	def ti(self): #Ti
	  if len(SpecificWorker.listaTargets) == 0 :
	      self.estado="INIT"
	      
	  else:
	      target=SpecificWorker.listaTargets.pop(0)
	      cercano=self.nodoCercano()
	      self.ruta=nx.shortest_path(g, cercano, target)
	      self.estado="PI"
	  
	  return 
	
	def pi(self):  #Pi
	  if len(self.ruta) == 0 : #lista vacia
	      estado="TI"
	  else:
	      self.target=self.ruta.pop(0)
	      x=self.posiciones[self.target][0]
	      y=self.posiciones[self.target][1]
	      self.gotopoint_proxy.go(str(self.target),x,y,0.0)
	      self.estado="GOI"
	  
	  return 

	def goi(self):
	  if self.gotopoint_proxy.atTarget() :
		print 'Nodo Alcanzado '+ str(self.target)
		self.estado='PI'
	  else :
		print 'Proximo nodo '+ str(self.target)
		time.sleep(1)
	  return

