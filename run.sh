#!/bin/bash

# Funzione per creare una nuova sessione screen e eseguire un comando
run_in_screen() {
    local session_name="$1"
    local command="$2"

    screen -dmS "$session_name" bash -c "$command; exec sh"
}

# Lancia Gazebo
run_in_screen "steer_bot_gazebo" "roslaunch steer_bot_gazebo steer_bot_sim.launch"
read -p "Premi [Enter] per continuare..."


# Lancia RViz
run_in_screen "steer_bot_viz" "roslaunch steer_bot_viz view_steer_bot_robot.launch"

read -p "Premi [Enter] per continuare..."


# Lancia design.py e attendi il completamento
#python3 ./steer_bot/design.py
#read -p "Premi [Enter] per continuare..."

# Informa l'utente sulla chiusura di rqt_robot_steering
echo "Se vuoi utilizzare il sistema di guida autonoma, chiudi la finestra rqt_robot_steering."

# Lancia il sistema di guida autonoma ROS nodes
run_in_screen "encoders" "rosrun odometry encoders.py"
run_in_screen "odom" "rosrun odometry odom.py"
run_in_screen "trajectory_drawing" "rosrun odometry trajectory_drawing.py"
run_in_screen "robot_localization" "roslaunch robot_localization ekf_template.launch"
run_in_screen "trajectory" "rosrun trajectory trajectory.py"
run_in_screen "pure_pursuit" "rosrun control pure_pursuit.py"

# Informa l'utente che tutti i processi sono stati avviati
echo "Tutti i processi sono stati avviati nelle sessioni screen."
