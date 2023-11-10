screen -ls | grep '(Detached)' | awk '{print $1}' | xargs -I {} screen -X -S {} quit
