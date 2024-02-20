import sys

def modify_python_file(file_path):
    try:
        with open(file_path, 'r') as file:
            file_data = file.read()
        
        if "math.sin(ix / 8.33) * ix / 4.0" not in file_data:  # Verifica se la modifica è già stata effettuata
            new_data = file_data.replace("math.sin(ix / 10.0) * ix / 5.0", "math.sin(ix / 8.33) * ix / 4.0")
            
            with open(file_path, 'w') as file:
                file.write(new_data)

            print(f"Modifica di {file_path} completata.")
        else:
            print(f"Modifica già applicata a {file_path}.")
    
    except Exception as e:
        print(f"Errore nella modifica del file Python: {e}")

def modify_launch_file(file_path):
    try:
        with open(file_path, 'r') as file:
            file_data = file.readlines()
        
        # Verifica se la modifica è già stata applicata per evitare duplicati
        if any("worlds/new_world.world" in line for line in file_data):
            print(f"Modifica già applicata a {file_path}.")
            return
        
        modified = False
        for i, line in enumerate(file_data):
            if '<arg name="world_name" default="worlds/empty_world" />' in line:
                file_data[i] = f"<!-- {line.strip()} -->\n"
                file_data.insert(i + 1, '  <arg name="world_name" default="worlds/new_world.world" />\n')
                modified = True
                break
        
        if modified:
            with open(file_path, 'w') as file:
                file.writelines(file_data)

            print(f"Modifica di {file_path} completata.")
        else:
            print(f"Nessuna modifica necessaria per {file_path}.")
    
    except Exception as e:
        print(f"Errore nella modifica del file launch: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: python modify_files.py <percorso-del-file-python> <percorso-del-file-launch>")
        sys.exit(1)
    
    python_file_path = sys.argv[1]
    launch_file_path = sys.argv[2]

    modify_python_file(python_file_path)
    modify_launch_file(launch_file_path)
