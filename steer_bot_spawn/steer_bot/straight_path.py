#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

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

#Funzione che richiama spawn_sdf_model
def spawn(name, x, y, z, model_content):
    spawn_model = rospy.ServiceProxy('steer_bot/gazebo/spawn_sdf_model', SpawnModel)
    x = float(x)
    y = float(y)
    z = float(z)
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    response = spawn_model(name, model_content, "", initial_pose, "world")
    rospy.loginfo(response.status_message)

#Funzione che crea il percorso rettilineo
def create_straight_path(length, percentage, num_cones):
    # Calcola la lunghezza effettiva in base alla percentuale
    actual_length = length * (percentage / 100.0)

    # Calcola lo spaziamento tra i coni
    spacing = actual_length / num_cones

    # Crea coordinate per il percorso destro
    right_path = [(i * spacing, 1) for i in range(num_cones)]

    # Crea coordinate per il percorso sinistro
    left_path = [(i * spacing, -1) for i in range(num_cones)]

    return right_path, left_path 

#Funzione principale
def node(percentage):
    
    rospy.init_node('spawn_model')
    rospy.wait_for_service('steer_bot/gazebo/spawn_sdf_model')

    #Definiamo la lunghezza di riferimento del percorso 
                             #Da regolare secondo la necessità, al 02/02/2024 ho inserito solo 13 coni
    reference_length = 13.0  #quindi la lunghezza del percorso sarà 13
                             #per un giusto funzionamento è consigliato inserire la stessa dimensione sia per reference_lenght che per num_cones
    
    #Numero di coni per ogni lato
    num_cones = 13

    #Inserisce le coordinate del percorso di destra e di sinistra in due liste
    right_path, left_path = create_straight_path(reference_length, percentage, num_cones)

    #Spawn dei coni
    try:
        for i, (x, y) in enumerate(right_path):
            spawn(f'pointB_{i}', x, y, 0, MODEL_CONTENT_CONE_BLUE)
        
        for i, (x, y) in enumerate(left_path):
            spawn(f'pointY_{i}', x, y, 0, MODEL_CONTENT_CONE_YELLOW)

    except rospy.ServiceException as exc:
        rospy.logerr("Errore durante l'invocazione del servizio: %s", str(exc))
    

#MAIN
if __name__ == '__main__':
    
    # Inseriamo la percentuale per ridimensionare il percorso
    while True:
        percentage = int(input("Inserisci un valore per ridimensionare il percorso (0-100%): "))
        if 0 <= percentage <= 100:
            break  
        else:
            print("Percentuale invalida. Perfavore inserisci una percentuale tra 0 e 100")

    #Modifica del file sdf
    MODEL_CONTENT_CONE_YELLOW = modify_and_read_sdf('./steer_bot/cone_yellow/model.sdf', percentage)
    MODEL_CONTENT_CONE_BLUE = modify_and_read_sdf('./steer_bot/cone_blue/model.sdf', percentage)
    MODEL_CONTENT_CONE_ORANGE = modify_and_read_sdf('./steer_bot/cone_orange/model.sdf', percentage)
    
    #Richiama la funzione principale
    node(percentage)