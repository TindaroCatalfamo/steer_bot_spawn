# steer_bot_spawn

Questo repository contiene una serie di script per la gestione di un progetto basato su Gazebo e ROS (Robot Operating System). Gli script sono progettati per semplificare l'installazione e l'avvio del progetto. Di seguito, una breve spiegazione degli script e come utilizzarli.

## Script di Installazione (`installation.sh`)

Lo script di installazione (`installation.sh`) è destinato a semplificare la distribuzione di modelli e risorse necessarie al progetto in Gazebo. Per utilizzarlo, seguire questi passaggi:

1. Aprire un terminale.

2. Navigare nella directory principale del progetto.

3. Eseguire il comando:

   ```bash
   ./installation.sh
   ```

   Questo script copierà le cartelle specificate nel percorso di destinazione predefinito (`$HOME/.gazebo/models`) per rendere le risorse disponibili per Gazebo.

## Script di Esecuzione (`run.sh`)

Lo script di esecuzione (`run.sh`) automatizza l'avvio del progetto Gazebo e dei nodi ROS associati. Per utilizzarlo, seguire questi passaggi:

1. Aprire un terminale.

2. Navigare nella directory principale del progetto.

3. Eseguire il comando:

   ```bash
   ./run.sh
   ```

   Questo script lancerà Gazebo e i nodi ROS associati in sessioni screen separate. Saranno richiesti ulteriori input durante l'esecuzione, quindi seguire le istruzioni fornite nel terminale.

## Script di Terminazione (`killscreen.sh`)

Lo script di terminazione (`killscreen.sh`) è utilizzato per chiudere le sessioni screen in esecuzione. Questo può essere utile quando si desidera terminare tutti i processi associati al progetto. Per utilizzarlo, seguire questi passaggi:

1. Aprire un terminale.

2. Navigare nella directory principale del progetto.

3. Eseguire il comando:

   ```bash
   ./killscreen.sh
   ```

   Questo script elencherà tutte le sessioni screen in esecuzione e chiederà conferma prima di chiuderle.

**Nota**: Assicurarsi di avere le dipendenze necessarie (Gazebo, ROS, ecc.) installate correttamente prima di eseguire gli script.
