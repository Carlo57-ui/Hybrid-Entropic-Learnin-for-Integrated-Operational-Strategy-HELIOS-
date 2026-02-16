import serial
import time
import math
import folium

# Configura el puerto serial
ser = serial.Serial("COM9", 115200, timeout=1)
time.sleep(2)

# Meta 
lat_meta = 19.512307       #CINVESTAV 19.511797     CASA 19.265987
lng_meta = -99.128923      #CINVESTAV  -99.129549   CASA -98.883518

# Función para calcular distancia Haversine en metros
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Radio de la Tierra en metros
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

# Enviar meta al TTGO_L
q_meta = f"{lat_meta},{lng_meta}\n"
print(f"Enviando q_meta: {q_meta.strip()}")
ser.write(q_meta.encode())

# Crear mapa inicial en la meta
mapa = folium.Map(location=[lat_meta, lng_meta], zoom_start=18)
folium.Marker([lat_meta, lng_meta], popup="Meta", icon=folium.Icon(color="red")).add_to(mapa)

trayectoria = []

while True:
    if ser.in_waiting > 0:
        response = ser.readline().decode(errors="ignore").strip()
        if response:
            try:
                lat_q, lng_q = map(float, response.split(","))
                delta_q = haversine(lat_meta, lng_meta, lat_q, lng_q)

                print(f"q: ({lat_q:.6f}, {lng_q:.6f}) | delta_q = {delta_q:.2f} m")

                trayectoria.append((lat_q, lng_q))

                if delta_q < 2:  # tolerancia de 2 metros
                    print("Meta temporal alcanzada")

                    # Dibujar la trayectoria final en el mapa
                    folium.PolyLine(trayectoria, color="blue", weight=2.5).add_to(mapa)
                    for punto in trayectoria:
                        folium.CircleMarker(punto, radius=2, color="blue", fill=True).add_to(mapa)

                    mapa.save("trayectoria.html")
                    print("Mapa guardado en trayectoria.html")
                    break

            except:
                print(f"Mensaje recibido: {response}")

    time.sleep(0.25)  # muestreo cada 250 ms




