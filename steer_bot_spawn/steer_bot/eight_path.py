#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy
import xml.etree.ElementTree as ET
import sympy as sp
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


#Funzione che genera il cerchio
def generate_circle(radius, center_y, center_x, num_points):
    coordinates_circle = []
    t = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(t) + center_x
    y = radius * np.sin(t) + center_y

    for i in range(num_points):
        coordinates_circle.append((x[i], y[i]))
    return coordinates_circle

#Funzione che risolve l'equazione del cerchio per trovare punti specifici per il piazzamento dei coni 
def solve_quadratic_equation(center_x, center_y, radius, y):
    # Definire le variabili
    x = sp.symbols('x')

    # Definire l'equazione
    equazione = (x - center_x)**2 + (y - center_y)**2 - (radius)**2

    # Espandere i quadrati
    equazione_espansa = sp.expand(equazione)

    # Riorganizzare l'equazione in forma canonica
    equazione_canonica = sp.collect(equazione_espansa, x)

    # Trovare le soluzioni
    soluzioni = sp.solve(equazione_canonica, x)
     
    return soluzioni

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

#Funzione di spawn
def spawn(name, x, y, model_content):
    spawn_model = rospy.ServiceProxy('steer_bot/gazebo/spawn_sdf_model', SpawnModel)
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y

    response = spawn_model(name, model_content, "", initial_pose, "world")
    rospy.loginfo(response.status_message)

#Funzione che spawna i coni su Gazebo
def node(blue_inner_circle, yellow_inner_circle, yellow_outer_circle, blue_outer_circle, percentage):
    rospy.init_node('spawn_model')
    rospy.wait_for_service('steer_bot/gazebo/spawn_sdf_model')
    
    #Array che contengono i coni sui cerchi esterni da non spawnare
    SKIP_Y_OUT = [0, 2, 3, 4, 5, 6]      
    SKIP_B_OUT = [0, 10, 11, 12, 13, 14]
    
    try:
        
        for i, (x, y) in enumerate(blue_inner_circle[:-1]):
            if i == 4:
                #Spawn coni intermedi gialli
                cone = solve_quadratic_equation(center_x, center_y1, radius2, y)
                spawn('point_MID_1', cone[0], y, MODEL_CONTENT_CONE_YELLOW)
                spawn('point_MID_2', cone[1], y, MODEL_CONTENT_CONE_YELLOW)
                #Spawn coni di partenza arancioni
                spawn('point_START_1', blue_inner_circle[i][0] - (5*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                spawn('point_START_2', blue_inner_circle[i][0] - (5.80*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                #Spawn coni di arrivo arancioni
                spawn('point_END_1', blue_inner_circle[i][0] + (5*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                spawn('point_END_2', blue_inner_circle[i][0] + (6*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)
                spawn('point_END_3', blue_inner_circle[i][0] + (7*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                spawn('point_END_4', blue_inner_circle[i][0] + (8*percentage/100), blue_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)  
                #Spawn coni intermedi arancioni grandi
                point_x = (blue_inner_circle[i-1][0] + blue_outer_circle[i][0])/2
                point_y = (blue_inner_circle[i-1][1] + blue_inner_circle[i][1])/2
                spawn('point_OR_1', point_x, point_y, MODEL_CONTENT_CONE_ORANGE_BIG)
                point_x = (blue_inner_circle[i+1][0] + blue_outer_circle[i][0])/2
                point_y = (blue_inner_circle[i+1][1] + blue_inner_circle[i][1])/2
                spawn('pointB_OR_2', point_x, point_y, MODEL_CONTENT_CONE_ORANGE_BIG)
            #Spawn coni cerchio interno blu
            spawn(f'pointB_IN_{i}', x, y, MODEL_CONTENT_CONE_BLUE) 
        
        for i, (x, y) in enumerate(yellow_inner_circle[:-1]):
            if i == 12:
                #Spawn coni intermedi blu
                cone = solve_quadratic_equation(center_x, center_y, radius2, y)
                spawn('point_MID_3', cone[0], y, MODEL_CONTENT_CONE_BLUE)
                spawn('point_MID_4', cone[1], y, MODEL_CONTENT_CONE_BLUE)
                #Spawn coni di partenza arancioni
                spawn('point_START_3', yellow_inner_circle[i][0] - (5*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                spawn('point_START_4', yellow_inner_circle[i][0] - (5.80*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)
                #Spawn coni di arrivo arancioni
                spawn('point_END_5', yellow_inner_circle[i][0] + (5*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE) 
                spawn('point_END_6', yellow_inner_circle[i][0] + (6*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)
                spawn('point_END_7', yellow_inner_circle[i][0] + (7*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)
                spawn('point_END_8', yellow_inner_circle[i][0] + (8*percentage/100), yellow_inner_circle[i][1], MODEL_CONTENT_CONE_ORANGE)
                #Spawn coni intermedi arancioni grandi
                point_x = (yellow_inner_circle[i-1][0] + yellow_outer_circle[i][0])/2
                point_y = (yellow_inner_circle[i-1][1] + yellow_inner_circle[i][1])/2
                spawn('point_OR_3', point_x, point_y, MODEL_CONTENT_CONE_ORANGE_BIG)
                point_x = (yellow_inner_circle[i+1][0] + yellow_outer_circle[i][0])/2
                point_y = (yellow_inner_circle[i+1][1] + yellow_inner_circle[i][1])/2
                spawn('point_OR_4', point_x, point_y, MODEL_CONTENT_CONE_ORANGE_BIG)
            #Spawn coni cerchio interno giallo
            spawn(f'pointY_IN_{i}', x, y, MODEL_CONTENT_CONE_YELLOW)

        #Spawn coni cerchio esterno giallo
        for i, (x, y) in enumerate(yellow_outer_circle):
            if i not in SKIP_Y_OUT:               
                spawn(f'pointY_OUT_{i}', x, y, MODEL_CONTENT_CONE_YELLOW)
        #Spawn coni cerchio esterno blu
        for i, (x, y) in enumerate(blue_outer_circle):
            if i not in SKIP_B_OUT:               
                spawn(f'pointB_OUT_{i}', x, y, MODEL_CONTENT_CONE_BLUE)
        
        #Spawn coni chiusura arrivo
        
                
    except rospy.ServiceException as exc:
        rospy.logerr("Error during service invocation: %s", str(exc))

#MAIN
if __name__ == "__main__":

    # Inseriamo la percentuale per ridimensionare il percorso
    while True:
        percentage = int(input("Inserisci un valore per ridimensionare il percorso (0-100%): "))
        if 0 <= percentage <= 100:
            break  
        else:
            print("Percentuale invalida. Perfavore inserisci una percentuale tra 0 e 100")
    
    #Modifica e lettura dei file SDF
    MODEL_CONTENT_CONE_YELLOW = modify_and_read_sdf('./steer_bot/cone_yellow/model.sdf', percentage)
    MODEL_CONTENT_CONE_BLUE = modify_and_read_sdf('./steer_bot/cone_blue/model.sdf', percentage)
    MODEL_CONTENT_CONE_ORANGE = modify_and_read_sdf('./steer_bot/cone_orange/model.sdf', percentage)
    MODEL_CONTENT_CONE_ORANGE_BIG = modify_and_read_sdf('./steer_bot/cone_orange_big/model.sdf', percentage)
    #Caratteristiche del percorso 
    radius1 = 2.5 * (percentage/100)  #Raggio dei cerchi interni
    radius2 = 5.0 * (percentage/100)  #Raggio dei cerchi esterni
    center_y = 3.75 * (percentage/100) #3.75
    center_y1 = -3.75 * (percentage/100) #3.75
    center_x = 6 * (percentage/100)
    num_points = 17 #Numero di punti utilizzato per generare il cerchio
    
    
    #Genera i cerchi interni e inserisce negli array le coordinate dei num_points dei cerchi interni
    blue_inner_circle = generate_circle(radius1, center_y1, center_x, num_points)
    yellow_inner_circle = generate_circle(radius1, center_y, center_x, num_points)
    #Genera i cerchi esterni e inserisce negli array le coordinate dei num_points dei cerchi esterni
    yellow_outer_circle = generate_circle(radius2, center_y1, center_x, num_points)
    blue_outer_circle= generate_circle(radius2, center_y, center_x, num_points)
    
    #Spawn dei coni
    node(blue_inner_circle, yellow_inner_circle, yellow_outer_circle, blue_outer_circle, percentage)

    
    '''#Plot dei cerchi
    blue_inner_x, blue_inner_y = zip(*blue_inner_circle)
    yellow_inner_x, yellow_inner_y = zip(*yellow_inner_circle)
    yellow_outer_x, yellow_outer_y = zip(*yellow_outer_circle)
    blue_outer_x, blue_outer_y = zip(*blue_outer_circle)
    plt.figure(figsize=(8, 8))
    plt.scatter(blue_inner_x, blue_inner_y, color='blue', label='Cerchio Blu Interno')
    plt.scatter(yellow_inner_x, yellow_inner_y, color='yellow', label='Cerchio Giallo Interno')
    plt.scatter(yellow_outer_x, yellow_outer_y, color='yellow', label='Cerchio Giallo Esterno')
    plt.scatter(blue_outer_x, blue_outer_y, color='blue', label='Cerchio Blu Esterno')
    plt.title('Visualizzazione dei Cerchi')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axhline(0, color='black',linewidth=0.5)
    plt.axvline(0, color='black',linewidth=0.5)
    plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)
    plt.legend()
    plt.axis('equal') 
    plt.show()'''
