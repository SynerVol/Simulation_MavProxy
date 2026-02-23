#!/bin/bash

# --- 1. Nettoyage préalable ---
echo "Arrêt des processus ArduPilot existants..."
pkill -f arducopter
pkill -f mavproxy
sleep 2

# --- 2. Configuration des chemins ---
ARDUPILOT_HOME="$HOME/code/drone/ardupilot"
SIM_DIR="$HOME/code/drone/Simulation_MavProxy/ardu-sim"
PARAM_DIR="$SIM_DIR/parameters"
LOG_DIR="$SIM_DIR/logs"

mkdir -p "$PARAM_DIR" "$LOG_DIR"
cd "$ARDUPILOT_HOME/ArduCopter"

echo "--------------------------------------------------"
echo "  SIMULATION ESSAIM : BATTERIE INFINIE ACTIVÉE  "
echo "--------------------------------------------------"

# --- 3. Préparation des paramètres (Batterie Infinie) ---
# On repart du fichier par défaut
cp "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm" "$PARAM_DIR/copter.parm"

# On ajoute les paramètres pour désactiver la gestion batterie
echo "BATT_MONITOR 0" >> "$PARAM_DIR/copter.parm"
echo "FS_BATT_ENABLE 0" >> "$PARAM_DIR/copter.parm"

# On prépare le fichier du drone 2 (copie du 1 + ID différent)
cp "$PARAM_DIR/copter.parm" "$PARAM_DIR/copter2.parm"
echo "SYSID_THISMAV 1" >> "$PARAM_DIR/copter.parm"
echo "SYSID_THISMAV 2" >> "$PARAM_DIR/copter2.parm"


# --- 4. Lancement du DRONE LEADER (SYSID 1) ---
echo "--> Lancement du LEADER (SYSID 1)"
# CORRECTION ICI : --add-param-file au lieu de --defaults
../Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f quad \
    --console \
    --map \
    -I0 \
    --sysid 1 \
    -L CMAC \
    --add-param-file="$PARAM_DIR/copter.parm" \
    --no-mavproxy &

sleep 5

# --- 5. Lancement du DRONE FOLLOWER (SYSID 2) ---
echo "--> Lancement du FOLLOWER (SYSID 2)"
# CORRECTION ICI : --add-param-file au lieu de --defaults
../Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f quad \
    -I1 \
    --sysid 2 \
    -L CMAC \
    --add-param-file="$PARAM_DIR/copter2.parm" \
    --no-mavproxy & 

sleep 5

# --- 6. Lancement de la Station Sol ---
echo "--> Démarrage MAVProxy"
mavproxy.py \
    --master tcp:127.0.0.1:5760 \
    --master tcp:127.0.0.1:5770 \
    --map \
    --console \
    --load-module='swarm'
