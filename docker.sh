#!/bin/bash

# --- 1. Nettoyage préalable ---
echo "Arrêt des processus ArduPilot existants..."
pkill -f arducopter || true
pkill -f mavproxy || true
sleep 2

# --- 2. Configuration des chemins (adaptés Docker) ---
ARDUPILOT_HOME="/ardupilot"
PARAM_DIR="/home/ardupilot/sim/parameters"
LOG_DIR="/home/ardupilot/sim/logs"

mkdir -p "$PARAM_DIR" "$LOG_DIR"

echo "--------------------------------------------------"
echo "  SIMULATION ESSAIM : BATTERIE INFINIE ACTIVÉE  "
echo "--------------------------------------------------"

# --- 3. Préparation des paramètres ---
cp "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm" "$PARAM_DIR/copter.parm"

echo "BATT_MONITOR 0"  >> "$PARAM_DIR/copter.parm"
echo "FS_BATT_ENABLE 0" >> "$PARAM_DIR/copter.parm"

cp "$PARAM_DIR/copter.parm" "$PARAM_DIR/copter2.parm"
echo "SYSID_THISMAV 1" >> "$PARAM_DIR/copter.parm"
echo "SYSID_THISMAV 2" >> "$PARAM_DIR/copter2.parm"

# --- 4. Lancement du LEADER (SYSID 1) ---
echo "--> Lancement du LEADER (SYSID 1)"
cd "$ARDUPILOT_HOME/ArduCopter"
"$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f quad \
    -I0 \
    --sysid 1 \
    -L CMAC \
    --add-param-file="$PARAM_DIR/copter.parm" \
    --no-mavproxy &

sleep 10  # plus long en Docker, démarrage plus lent

# --- 5. Lancement du FOLLOWER (SYSID 2) ---
echo "--> Lancement du FOLLOWER (SYSID 2)"
"$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f quad \
    -I1 \
    --sysid 2 \
    -L CMAC \
    --add-param-file="$PARAM_DIR/copter2.parm" \
    --no-mavproxy &

sleep 10

# --- 6. Attendre que les ports soient prêts ---
echo "--> Attente des ports MAVLink..."
while ! nc -z 127.0.0.1 5760; do sleep 1; done
echo "Port 5760 prêt"
while ! nc -z 127.0.0.1 5770; do sleep 1; done
echo "Port 5770 prêt"

# --- 7. Lancement MAVProxy ---
echo "--> Démarrage MAVProxy"
/home/ardupilot/venv/bin/mavproxy.py \
    --master tcp:127.0.0.1:5760 \
    --master tcp:127.0.0.1:5770 \
    --map \
    --console \
    --load-module='swarm'
