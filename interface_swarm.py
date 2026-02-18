import collections
import collections.abc
# Patch Python 3.10+
collections.MutableMapping = collections.abc.MutableMapping
collections.Iterable = collections.abc.Iterable

import customtkinter as ctk
import tkintermapview
from dronekit import connect, Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import math
import threading
import time

# --- CONFIGURATION ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

# Ports
LEADER_PORT = 'tcp:127.0.0.1:5763'
FOLLOWER_PORT = 'tcp:127.0.0.1:5773'

# =============================================================================
# BACKEND (LOGIQUE DRONE - INCHANGÉ)
# =============================================================================
class SwarmBackend:
    def __init__(self):
        self.leader = None
        self.follower = None
        self.connected = False
        self.target_location = None 

    def connect_drones(self):
        try:
            print("Backend: Connexion au Leader...")
            self.leader = connect(LEADER_PORT, wait_ready=True)
            print("Backend: Connexion au Follower...")
            self.follower = connect(FOLLOWER_PORT, wait_ready=True)
            self.connected = True
            return True
        except Exception as e:
            print(f"Backend Erreur: {e}")
            return False

    def get_drone_positions(self):
        pos = {}
        try:
            if self.leader:
                loc = self.leader.location.global_relative_frame
                if loc.lat: pos['leader'] = (loc.lat, loc.lon)
            if self.follower:
                loc = self.follower.location.global_relative_frame
                if loc.lat: pos['follower'] = (loc.lat, loc.lon)
        except: pass 
        return pos

    def emergency_stop(self):
        print("!!! ARRÊT D'URGENCE DÉCLENCHÉ !!!")
        if self.leader: self.leader.mode = VehicleMode("RTL")
        if self.follower: self.follower.mode = VehicleMode("RTL")

    def get_location_metres(self, original_location, dNorth, dEast, alt):
        earth_radius = 6378137.0 
        dLat = dNorth/earth_radius * 180/math.pi
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180)) * 180/math.pi
        return LocationGlobalRelative(original_location.lat + dLat, original_location.lon + dLon, alt)

    def generate_mission_points(self, center_lat, center_lon, radius, spacing, altitude, side="left"):
        mission_items = []
        center_loc_obj = type('obj', (object,), {'lat': center_lat, 'lon': center_lon, 'alt': altitude})
        mission_items.append(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        y = -radius 
        direction = 1 
        while y <= radius:
            if abs(y) >= radius:
                y += spacing
                continue
            x_span = math.sqrt(radius**2 - y**2)
            if side == "left": x_min, x_max = -x_span, 0
            else: x_min, x_max = 0, x_span   
            p1 = self.get_location_metres(center_loc_obj, y, x_min if direction == 1 else x_max, altitude)
            p2 = self.get_location_metres(center_loc_obj, y, x_max if direction == 1 else x_min, altitude)
            mission_items.append(p1)
            mission_items.append(p2)
            y += spacing
            direction *= -1
        return mission_items

    def upload_mission_to_vehicle(self, vehicle, waypoints, altitude):
        cmds = vehicle.commands
        cmds.clear()
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))
        for wp in waypoints:
            if isinstance(wp, LocationGlobalRelative):
                cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp.lat, wp.lon, altitude))
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        cmds.upload()

    def start_swarm_mission(self, center_lat, center_lon, radius, alt, fov):
        if not self.connected: return
        largeur_vue = 2 * alt * math.tan(math.radians(fov / 2))
        espacement = largeur_vue * 0.8 
        print("Backend: Calcul des trajectoires...")
        wps1 = self.generate_mission_points(center_lat, center_lon, radius, espacement, alt, "left")
        wps2 = self.generate_mission_points(center_lat, center_lon, radius, espacement, alt, "right")
        self.upload_mission_to_vehicle(self.leader, wps1, alt)
        self.upload_mission_to_vehicle(self.follower, wps2, alt)
        
        print("Backend: Décollage Drone 1...")
        self.leader.mode = VehicleMode("GUIDED")
        self.leader.armed = True
        while not self.leader.armed: time.sleep(0.1)
        self.leader.simple_takeoff(5)
        print("Backend: Décollage Drone 2...")
        self.follower.mode = VehicleMode("GUIDED")
        self.follower.armed = True
        while not self.follower.armed: time.sleep(0.1)
        self.follower.simple_takeoff(5)
        print("Backend: Attente stabilisation...")
        time.sleep(6) 
        print("Backend: Passage en AUTO...")
        self.leader.mode = VehicleMode("AUTO")
        self.follower.mode = VehicleMode("AUTO")

# =============================================================================
# FRONTEND AMÉLIORÉ (SÉLECTEUR DE CARTE)
# =============================================================================
class SwarmApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.backend = SwarmBackend() 
        
        self.title("SYNERVOL - Ground Control Station")
        self.geometry("1200x800")

        # --- Layout ---
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- Menu Gauche ---
        self.frame_left = ctk.CTkFrame(self, width=260, corner_radius=0)
        self.frame_left.grid(row=0, column=0, sticky="nswe")

        self.lbl_title = ctk.CTkLabel(self.frame_left, text="MISSION CONTROL", font=ctk.CTkFont(size=20, weight="bold"))
        self.lbl_title.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Connexion
        self.btn_connect = ctk.CTkButton(self.frame_left, text="CONNECTER DRONES", command=self.on_connect_click, fg_color="#107C10")
        self.btn_connect.grid(row=1, column=0, padx=20, pady=10)

        # --- NOUVEAU : SÉLECTEUR DE CARTE ---
        self.lbl_map = ctk.CTkLabel(self.frame_left, text="Type de Carte :", text_color="gray")
        self.lbl_map.grid(row=2, column=0, padx=20, pady=(10, 0), sticky="w")
        
        self.option_map = ctk.CTkOptionMenu(self.frame_left, 
                                            values=["Satellite (Google)", "Plan (Normal)"],
                                            command=self.change_map_layer)
        self.option_map.grid(row=3, column=0, padx=20, pady=5)
        # ------------------------------------

        self.lbl_params = ctk.CTkLabel(self.frame_left, text="Paramètres de zone:", text_color="gray")
        self.lbl_params.grid(row=4, column=0, padx=20, pady=(20, 5), sticky="w")

        self.entry_radius = ctk.CTkEntry(self.frame_left, placeholder_text="Rayon (m)")
        self.entry_radius.grid(row=5, column=0, padx=20, pady=5)
        self.entry_radius.insert(0, "100")

        self.entry_alt = ctk.CTkEntry(self.frame_left, placeholder_text="Altitude (m)")
        self.entry_alt.grid(row=6, column=0, padx=20, pady=5)
        self.entry_alt.insert(0, "20")

        self.btn_start = ctk.CTkButton(self.frame_left, text="LANCER MISSION", command=self.on_start_click, state="disabled", fg_color="#005FB8")
        self.btn_start.grid(row=7, column=0, padx=20, pady=(30, 10))

        self.btn_clear = ctk.CTkButton(self.frame_left, text="NETTOYER CARTE", command=self.action_clear_map, fg_color="gray", hover_color="#444")
        self.btn_clear.grid(row=8, column=0, padx=20, pady=10)

        self.btn_stop = ctk.CTkButton(self.frame_left, text="STOP URGENCE (RTL)", command=self.on_stop_click, fg_color="#C42B1C", hover_color="#8E1F14", height=50)
        self.btn_stop.grid(row=10, column=0, padx=20, pady=50, sticky="s")
        self.frame_left.grid_rowconfigure(10, weight=1)

        # --- Carte ---
        self.map_widget = tkintermapview.TkinterMapView(self, corner_radius=0)
        self.map_widget.grid(row=0, column=1, sticky="nswe")
        self.map_widget.set_position(-35.363261, 149.165230)
        self.map_widget.set_zoom(17)
        self.map_widget.add_right_click_menu_command(label="Définir Zone ici", command=self.set_zone_marker, pass_coords=True)
        
        # Initialisation par défaut (Satellite)
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)

        # Bouton Recentrer
        self.overlay_frame = ctk.CTkFrame(self.map_widget, width=50, height=50, fg_color="transparent")
        self.overlay_frame.place(relx=0.98, rely=0.02, anchor="ne")
        self.btn_recenter = ctk.CTkButton(self.overlay_frame, text="Recentrer", width=80, height=30, command=self.action_recenter, fg_color="#333")
        self.btn_recenter.pack()

        # Objets Graphiques
        self.marker_target = None
        self.polygon_zone = None
        self.drone1_marker = None
        self.drone2_marker = None
        self.path_coords_1 = []
        self.path_coords_2 = []
        self.path_line_1 = None
        self.path_line_2 = None

        self.update_telemetry()

    # --- NOUVELLE FONCTION : CHANGEMENT DE CARTE ---
    def change_map_layer(self, new_map):
        if new_map == "Satellite (Google)":
            # Google Satellite
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        elif new_map == "Plan (Normal)":
            # Google Maps Normal
            self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)

    # --- LE RESTE DES FONCTIONS EST IDENTIQUE ---
    def on_connect_click(self):
        threading.Thread(target=self.thread_connect).start()

    def thread_connect(self):
        self.btn_connect.configure(text="Connexion...", state="disabled")
        if self.backend.connect_drones():
            self.btn_connect.configure(text="CONNECTÉ", fg_color="green", state="disabled")
            self.btn_start.configure(state="normal")
        else:
            self.btn_connect.configure(text="ÉCHEC", fg_color="red", state="normal")

    def set_zone_marker(self, coords):
        self.backend.target_location = coords
        if self.marker_target: self.marker_target.delete()
        if self.polygon_zone: self.polygon_zone.delete()
        self.marker_target = self.map_widget.set_marker(coords[0], coords[1], text="Cible")
        try:
            radius = float(self.entry_radius.get())
            points = []
            earth_radius = 6378137.0
            for angle in range(0, 361, 10):
                dNorth = radius * math.cos(math.radians(angle))
                dEast = radius * math.sin(math.radians(angle))
                dLat = dNorth/earth_radius * 180/math.pi
                dLon = dEast/(earth_radius*math.cos(math.pi*coords[0]/180)) * 180/math.pi
                points.append((coords[0] + dLat, coords[1] + dLon))
            self.polygon_zone = self.map_widget.set_polygon(points, fill_color="red", outline_color="red", border_width=2)
        except: pass

    def on_start_click(self):
        if not self.backend.target_location: return
        r = float(self.entry_radius.get())
        alt = float(self.entry_alt.get())
        threading.Thread(target=self.backend.start_swarm_mission, 
                         args=(self.backend.target_location[0], self.backend.target_location[1], r, alt, 60)).start()

    def on_stop_click(self):
        self.backend.emergency_stop()

    def action_clear_map(self):
        if self.path_line_1: self.path_line_1.delete()
        if self.path_line_2: self.path_line_2.delete()
        if self.polygon_zone: self.polygon_zone.delete()
        if self.marker_target: self.marker_target.delete()
        self.path_coords_1 = []
        self.path_coords_2 = []
        self.path_line_1 = None
        self.path_line_2 = None
        self.marker_target = None
        self.backend.target_location = None
        print("Carte nettoyée.")

    def action_recenter(self):
        pos = self.backend.get_drone_positions()
        if 'leader' in pos:
            self.map_widget.set_position(pos['leader'][0], pos['leader'][1])
            self.map_widget.set_zoom(18)
        elif self.backend.target_location:
            self.map_widget.set_position(self.backend.target_location[0], self.backend.target_location[1])
            self.map_widget.set_zoom(17)
        else:
            print("Rien à centrer.")

    def update_telemetry(self):
        positions = self.backend.get_drone_positions()
        if 'leader' in positions:
            lat, lon = positions['leader']
            if self.drone1_marker: self.drone1_marker.set_position(lat, lon)
            else: self.drone1_marker = self.map_widget.set_marker(lat, lon, text="L", marker_color_circle="blue")
            self.path_coords_1.append((lat, lon))
            if len(self.path_coords_1) > 2:
                if self.path_line_1: self.path_line_1.delete()
                self.path_line_1 = self.map_widget.set_path(self.path_coords_1, color="blue", width=2)

        if 'follower' in positions:
            lat, lon = positions['follower']
            if self.drone2_marker: self.drone2_marker.set_position(lat, lon)
            else: self.drone2_marker = self.map_widget.set_marker(lat, lon, text="F", marker_color_circle="green")
            self.path_coords_2.append((lat, lon))
            if len(self.path_coords_2) > 2:
                if self.path_line_2: self.path_line_2.delete()
                self.path_line_2 = self.map_widget.set_path(self.path_coords_2, color="green", width=2)

        self.after(500, self.update_telemetry)

if __name__ == "__main__":
    app = SwarmApp()
    app.mainloop()
