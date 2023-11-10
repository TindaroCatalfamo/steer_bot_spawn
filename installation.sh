#!/bin/bash

# Chiedere i permessi di superutente
if [ "$EUID" -ne 0 ]; then
  echo "Per favore esegui lo script come superutente (sudo)."
  exit
fi

# Cartelle da spostare
folders=(cone asphalt_plane grass mat oak_tree)

# Directory di destinazione
dest_folder="$HOME/.gazebo/models"

# Entrare nella cartella steer_bot
cd steer_bot || { echo "Cartella steer_bot non trovata"; exit 1; }

# Sposta le cartelle
for folder in "${folders[@]}"; do
  if [ -d "$folder" ]; then
    mv "$folder" $dest_folder/
    echo "Spostamento di $folder completato."
  else
    echo "La cartella $folder non esiste."
  fi
done

# Verifica e crea la cartella worlds se non esiste
worlds_folder="/usr/share/gazebo-11/worlds"
if [ ! -d "$worlds_folder" ]; then
  mkdir -p $worlds_folder
  echo "Cartella $worlds_folder creata."
fi


echo "Finito."
