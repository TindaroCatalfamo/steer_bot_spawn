#!/usr/bin/env python

import numpy as np
from scipy.special import binom
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.interpolate import splprep, splev
import math
from scipy.spatial import Delaunay
import time
from scipy.spatial.distance import cdist
import rospy
import random
import xml.etree.ElementTree as ET
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

#Classe per rappresentare un segmento di curva Bezier
class Segment():
	def __init__(self, p1, p2, angle1, angle2, **kw):
		self.p1 = p1; self.p2 = p2
		self.angle1 = angle1; self.angle2 = angle2
		self.numpoints = kw.get("numpoints", 100)
		r = kw.get("r", 0.3)
		d = np.sqrt(np.sum((self.p2-self.p1)**2))
		self.r = r*d
		self.p = np.zeros((4,2))
		self.p[0,:] = self.p1[:]
		self.p[3,:] = self.p2[:]
		self.calc_intermediate_points(self.r)

	def calc_intermediate_points(self,r):
		self.p[1,:] = self.p1 + np.array([self.r*np.cos(self.angle1),
									self.r*np.sin(self.angle1)])
		self.p[2,:] = self.p2 + np.array([self.r*np.cos(self.angle2+np.pi),
									self.r*np.sin(self.angle2+np.pi)])
		self.curve = bezier(self.p,self.numpoints)

def modify_and_read_sdf(filename, percentage):
    # Funzione interna per la modifica della dimensione dei coni
    def modify_sdf_scale(filename, percentage):
        tree = ET.parse(filename)
        root = tree.getroot()

        # Trova l'elemento <scale> e lo modifica
        for scale_elem in root.iter('scale'):
            scale_elem.text = f"{1*(percentage/100)} {1*(percentage/100)} {1*(percentage/100)}"

        # Salva il file SDF modificato
        tree.write(filename, encoding="UTF-8", xml_declaration=True)

    # Modifica il file SDF
    modify_sdf_scale(filename, percentage)

    # Leggi il contenuto del file SDF modificato
    with open(filename, 'r') as file:
        sdf_content_modified = file.read()

    return sdf_content_modified

#Funzione per calcolcare una curva Bezier
def bezier(points, num=200):
	N = len(points)
	t = np.linspace(0, 1, num=num)
	curve = np.zeros((num, 2))
	for i in range(N):
		curve += np.outer(bernstein(N - 1, i, t), points[i])
	return curve

#Funzione per ottenere una curva da una serie di punti 
def get_curve(points, **kw):
	segments = []
	for i in range(len(points)-1):
		seg = Segment(points[i,:2], points[i+1,:2], points[i,2],points[i+1,2],**kw)
		segments.append(seg)
	curve = np.concatenate([s.curve for s in segments])
	return segments, curve

#Funzione per ordinare una serie di punti in senso orario
def ccw_sort(p):
	d = p-np.mean(p,axis=0)
	s = np.arctan2(d[:,0], d[:,1])
	return p[np.argsort(s),:]

#Funzione per ottenere una curva Bezier con punti equidistanti
def get_bezier_curve(a, rad=0.2, edgy=0):
	# given an array of points *a*, create a curve through
	#those points. 
	#*rad* is a number between 0 and 1 to steer the distance of
	#	  control points.
	#*edgy* is a parameter which controls how "edgy" the curve is,
	#	   edgy=0 is smoothest
	p = np.arctan(edgy)/np.pi+.5
	a = ccw_sort(a)
	a = np.append(a, np.atleast_2d(a[0,:]), axis=0)
	d = np.diff(a, axis=0)
	ang = np.arctan2(d[:,1],d[:,0])
	f = lambda ang : (ang>=0)*ang + (ang<0)*(ang+2*np.pi)
	ang = f(ang)
	ang1 = ang
	ang2 = np.roll(ang,1)
	ang = p*ang1 + (1-p)*ang2 + (np.abs(ang2-ang1) > np.pi )*np.pi
	ang = np.append(ang, [ang[0]])
	a = np.append(a, np.atleast_2d(ang).T, axis=1)
	s, c = get_curve(a, r=rad, method="var")
	x,y = c.T
	return x,y, a

#Funzione per ottenere punti casuali su una curva
def get_random_points(n=5, scale=0.8, mindst=None, rec=0):
	""" create n random points in the unit square, which are *mindst*
	apart, then scale them."""
	mindst = mindst or .7/n
	a = np.random.rand(n,2)
	d = np.sqrt(np.sum(np.diff(ccw_sort(a), axis=0), axis=1)**2)
	if np.all(d >= mindst) or rec>=200:
		return a*scale
	else:
		return get_random_points(n=n, scale=scale, mindst=mindst, rec=rec+1)

bernstein = lambda n, k, t: binom(n,k)* t**k * (1.-t)**(n-k)

#Funzione per generare la traiettoria della pista
def generate_track(equidistant_u, equidistant_point_samples, tck, distance=1.8):
	points_list_right = []
	points_list_left = []
	orange = True 
	for i in range (len(equidistant_u[0])-1):
	#for i in range (0, 17, 1):
		u0 = equidistant_u[0][i]
		x0 = equidistant_point_samples[0][0][i]
		y0 = equidistant_point_samples[1][0][i]
		dx,dy = interpolate.splev(u0,tck,der=1)	
		#tngnt = lambda x: dydx*x + (y0-dydx*x0)
		der = dy/dx
		#print("u0 = ", u0, "x0, y0 = ", x0, y0, "dy, dx, der = ", dy, dx, der)
		if(der >= 0):
			#print("first case, i = ", i)
			x1 = x0 - distance * np.sin(np.arctan(der))
			y1 = y0 + distance * np.cos(np.arctan(der))
			x2 = x0 + distance * np.sin(np.arctan(der))
			y2 = y0 - distance * np.cos(np.arctan(der))
			#print("x1, y1 = ", x1, y1, "x2, y2 =", x2, y2)
		elif(der < 0):
			#print("second case, i =", i)
			x1 = x0 + distance * np.sin(abs(np.arctan(der)))
			y1 = y0 + distance * np.cos(abs(np.arctan(der)))
			x2 = x0 - distance * np.sin(abs(np.arctan(der)))
			y2 = y0 - distance * np.cos(abs(np.arctan(der)))
			#print("x1, y1 = ", x1, y1, "x2, y2 =", x2, y2)
		#plt.plot(x0,y0, marker="x", markersize=10, markeredgecolor="red", markerfacecolor="red")
		if orange == True:
			plt.plot(x1,y1, marker="o", markersize=5, markeredgecolor="orange", markerfacecolor="orange")
			plt.plot(x2,y2, marker="o", markersize=5, markeredgecolor="orange", markerfacecolor="orange")
			points_list_left.append([x1,y1])
			points_list_right.append([x2,y2])
			orange = False
		else:
			if(dx < 0):
				plt.plot(x1,y1, marker="o", markersize=5, markeredgecolor="blue", markerfacecolor="blue")
				plt.plot(x2,y2, marker="o", markersize=5, markeredgecolor="yellow", markerfacecolor="yellow")
				points_list_left.append([x1,y1])
				points_list_right.append([x2,y2])
			else:
				plt.plot(x1,y1, marker="o", markersize=5, markeredgecolor="yellow", markerfacecolor="yellow")
				plt.plot(x2,y2, marker="o", markersize=5, markeredgecolor="blue", markerfacecolor="blue")
				points_list_left.append([x2,y2])
				points_list_right.append([x1,y1])
		i = i + 1
	#print(points_list_left)
	len_left = len(points_list_left)
	#print(points_list_right)
	len_right = len(points_list_right)
	#print(points_list)
	points = np.array(points_list_right)
	points = np.append(points, points_list_left, axis=0)
	#points = np.array(points_list)
	#print(points)
	return points, len_left, len_right, points_list_left, points_list_right

#Funzione per rimuovere punti ripetuti in una curva
def clean_curve(x, y):
	to_be_deleted = []
	for i in range(len(x)-1):
		if(math.isclose(x[i], x[i+1])):
			#print(x[i], ",", y[i], " == ", x[i+1], ",", y[i+1], "i = ", i)
			to_be_deleted.append(i)
			
	j=0
	for i in to_be_deleted:
		#print("deleting element ", i)
		x = np.delete(x, i-j)
		y = np.delete(y, i-j)
		j=j+1
	return x, y

#Funzione per ottenere punti equidistanti lungo una curva
def equally_spaced_curve(x, y):
	tck, _ = splprep([x, y], s=0)
	n_points_spline = 70
	u = np.linspace(0, 1, n_points_spline)
	sampled_points = splev(u, tck)
	sampled_points = np.stack(sampled_points, axis=-1)
	inter_point_differences = np.diff(sampled_points, axis=0)
	inter_point_distances = np.linalg.norm(inter_point_differences, axis=-1)
	cumulative_distance = np.cumsum(inter_point_distances)
	cumulative_distance /= cumulative_distance[-1]
	tck_prime, _ = splprep([np.linspace(0, 1, num=len(cumulative_distance))], u=cumulative_distance, s=0, k=3)
	equidistant_u = splev(u, tck_prime)
	equidistant_point_samples = splev(equidistant_u, tck)
	equidistant_u[0] = np.delete(equidistant_u[0], 0, 0)
	equidistant_point_samples[0] = np.delete(equidistant_point_samples[0], 0, 1)
	equidistant_point_samples[1] = np.delete(equidistant_point_samples[1], 0, 1)
	return tck, equidistant_u, equidistant_point_samples

#Funzione per eseguire la triangolazione di Delaunay su una serie di punti
def delaunay_triangulation(points, len_left, len_right):
	tri = Delaunay(points)

	delaunayEdges = []
	for simp in tri.simplices:
		for i in range(3):
			j = i + 1
			if j == 3:
				j = 0
			if((simp[i]< len_left and simp[j] > len_left) or (simp[i] > len_left and simp[j] < len_left)):
				delaunayEdges.append([[points[simp[i]][0],points[simp[i]][1]],[points[simp[j]][0],points[simp[j]][1]]])

	middlepoints = []

	for edge in delaunayEdges:
		middlepoints.append([(edge[0][0]+edge[1][0])/2, (edge[0][1]+edge[1][1])/2])

	filtered_middlepoints = []

	[filtered_middlepoints.append(item) for item in middlepoints if item not in filtered_middlepoints]

	return delaunayEdges, filtered_middlepoints

#Funzione per trovare il punto più vicino a un dato nodo in una lista di nodi
def closest_node(node, nodes):
	return nodes[cdist([node], nodes).argmin()], cdist([node], nodes).argmin()

#Funzione per ordinare una lista di punti in base alla loro vicinanza a un punto iniziale
def order_middlepoints(initial_point, middlepoints):
	counter = 0
	ordered_middlepoints = []
	for i in range(len(middlepoints)):
		initial_point, indice = closest_node(initial_point, middlepoints)
		point = middlepoints.pop(indice)
		ordered_middlepoints.append(point)
		counter += 1
	return ordered_middlepoints

#Funzione per creare un modello su Gazebo
def spawn(name, x, y, z, model_content):
    spawn_model = rospy.ServiceProxy('steer_bot/gazebo/spawn_sdf_model', SpawnModel)
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    response = spawn_model(name, model_content, "", initial_pose, "world")
    rospy.loginfo(response.status_message)

#Funzione che spawna i coni su Gazebo
def node():
    rospy.init_node('spawn_model')
    rospy.wait_for_service('steer_bot/gazebo/spawn_sdf_model')

    try:
        # Genera i coni 
        for i, (x, y) in enumerate(points_list_left):
            spawn(f'pointL_{i}', x, y, 0.3, MODEL_CONTENT_CONE_LEFT)
			
        for i, (x, y) in enumerate(points_list_right):
            spawn(f'pointR_{i}', x, y, 0.3, MODEL_CONTENT_CONE_RIGHT)

    except rospy.ServiceException as exc:
        rospy.logerr("Error during service invocation: %s", str(exc))

#Funzione MAIN
if __name__ == "__main__":
	
	while True:
		percentage= int(input("Inserisci un valore per ridimensionare il percorso (0-100%): "))
		if 0 <= percentage <= 100:
			break
		else: 
			print("Percentuale invalida. Perfavore inserisci una percentuale tra 0 e 100")
		
        
    #Modifica e lettura dei file SDF
	MODEL_CONTENT_CONE_LEFT = modify_and_read_sdf('./steer_bot/cone_yellow/model.sdf', percentage)
	MODEL_CONTENT_CONE_RIGHT = modify_and_read_sdf('./steer_bot/cone_blue/model.sdf', percentage)

	
	#Caratteristiche del percorso
	n=6
	rad = 0.3
	edgy = 0.1
	scale = 70*(percentage/100)
	mindst = 150*(percentage/100)
	distance = 2*(percentage/100)
	#Genera una curva casuale
	a = get_random_points(n, scale, mindst, 0)
	
	#a = [[17.06326744,18.7787095 ],[66.3086311  ,30.46364876],[17.4845151 , 21.68553352],[41.1450949 , 19.73494244],[34.96106364, 55.57534602]]
	print("Questa è la mia curva: " + str(a) + "\n")
	
	#force a straight line near [0,0]
	#a = np.insert(a, [0], [[3,0]], axis=0)
	#a = np.insert(a, [0], [[2,0]], axis=0)
	#a = np.insert(a, [0], [[1,0]], axis=0)
	#a = np.insert(a, [0], [[0,0]], axis=0)
	#a = np.insert(a, [0], [[-1,0]], axis=0)
	#a = np.insert(a, [0], [[-2,0]], axis=0)

	
	#get the track curve and remove repeated points
	x, y, _ = get_bezier_curve(a,rad, edgy)
	x, y = clean_curve(x, y)

	#understands the dimensions of the final figure without showing it and then clear
	plt.figure(figsize=(8, 8))
	plt.axis("equal")
	plt.plot(*zip(*a), 'go')
	plt.plot(x,y, 'green', linestyle="-")
	xmin, xmax, ymin, ymax = plt.axis()
	plt.clf()
	plt.xlim([xmin - 20, xmax + 20])
	plt.ylim([ymin - 20, ymax + 20])

	
	#now plot the starting points and the Bezier curve
	plt.plot(*zip(*a), 'go') #points
	plt.plot(x,y, 'green', linestyle="-") #curve
	plt.show(block = False)
	input("Press Enter to continue...")

	#now get equally spaced points on the track curve with equally_space_curve and plot them
	tck, equidistant_u, equidistant_point_samples = equally_spaced_curve(x, y)
	xmin, xmax, ymin, ymax = plt.axis()
	plt.xlim([xmin, xmax])
	plt.ylim([ymin, ymax])
	plt.plot(equidistant_point_samples[0][0][0], equidistant_point_samples[1][0][0], marker="x", markersize=10, markeredgecolor="red", markerfacecolor="blue")
	plt.plot(equidistant_point_samples[0], equidistant_point_samples[1], marker="o", markersize=2, markeredgecolor="blue", markerfacecolor="blue")
	plt.show(block = False)
	input("Press Enter to continue...")

	#generate and plot the blue and yellow cones 
	points, len_left, len_right, points_list_left, points_list_right = generate_track(equidistant_u, equidistant_point_samples, tck, distance)
	plt.show(block = False)
	input("Press Enter to continue...")

	#Inizia lo spawn dei coni su Gazebo 
	node()
	#Chiusura dello script
	input("Press Enter to close the script...")