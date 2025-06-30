import asyncio
import serial
import websockets
import json
import random
import time
import traceback
from pathlib import Path # Para manejar rutas de archivos
from aiohttp import web # <--- IMPORTANTE: Importar aiohttp
from motor.motor_asyncio import AsyncIOMotorClient # pip install motor pymongo
from datetime import datetime

# --- Configuración MongoDB ---
MONGODB_URI = "mongodb://localhost:27017"  # Cambia si tu MongoDB está en otro servidor
MONGODB_DB_NAME = "telemetria_boya"
MONGODB_COLLECTION_NAME = "datos_sensores"

# --- Configuración General ---
USE_SIMULATED_DATA = True  # CAMBIA ESTO: True para simular, False para usar el puerto serie
APP_NAME = "Monitor de Boya de Telemetría"

# --- Configuración HTTP Server (NUEVO/MODIFICADO) ---
HTTP_HOST = '0.0.0.0'  # Escuchar en todas las interfaces de red (accesible en tu LAN)
                       # Usa 'localhost' si solo quieres acceso desde tu propia PC.
HTTP_PORT = 8000       # Puerto para el servidor HTTP (donde verás el HTML)
HTML_FILENAME = "boya_dashboard.html" # Nombre de tu archivo HTML principal

# --- Configuración WebSocket ---
WEBSOCKET_HOST = 'localhost' # El servidor WebSocket sigue escuchando en localhost
WEBSOCKET_PORT = 8765        # Puerto para el WebSocket (el HTML se conecta a este)

# --- Configuración Serial (solo se usa si USE_SIMULATED_DATA es False) ---
SERIAL_PORT = 'COM7'  # MODIFICA ESTO
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1

# --- Configuración de Simulación (solo se usa si USE_SIMULATED_DATA es True) ---
SIMULATION_INTERVAL = 1.0

# --- Lógica de WebSocket (sin cambios significativos) ---
clients = set()
if USE_SIMULATED_DATA:
    random.seed(time.time())
    
# --- Variables Globales ---
mongo_client = None
db = None


async def register_client(websocket):
    clients.add(websocket)
    client_ip_port = str(websocket.remote_address)
    print(f"Cliente WS [{client_ip_port}] conectado. Total: {len(clients)}")

async def unregister_client(websocket):
    if websocket in clients:
        clients.remove(websocket)
        client_ip_port = str(websocket.remote_address)
        print(f"Cliente WS [{client_ip_port}] desconectado. Total: {len(clients)}")

async def send_to_all_clients(message):
    if clients:
        current_clients = list(clients)
        # Usamos return_exceptions=True para que si un envío falla, no detenga los demás.
        results = await asyncio.gather(*[client.send(message) for client in current_clients], return_exceptions=True)
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                # Podríamos intentar remover al cliente que falló
                # print(f"Error al enviar a cliente {current_clients[i].remote_address}: {result}")
                # await unregister_client(current_clients[i]) # Opcional: desregistrar si falla el envío
                pass


async def websocket_connection_handler(websocket): # Asegúrate que 'path' esté aquí
    client_ip_port = str(websocket.remote_address)
    await register_client(websocket)
    try:
        async for message_from_client in websocket:
            print(f"[{client_ip_port}] WS Handler: Mensaje recibido (ignorado): {message_from_client}")
            pass
        print(f"[{client_ip_port}] WS Handler: Bucle 'async for' terminado.")
    except websockets.exceptions.ConnectionClosedOK:
        print(f"[{client_ip_port}] WS Handler: ConnectionClosedOK.")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"[{client_ip_port}] WS Handler: ConnectionClosedError: Código={e.code}, Razón='{e.reason}'")
    except Exception as e:
        print(f"[{client_ip_port}] WS Handler: ERROR INESPERADO: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        print(f"[{client_ip_port}] WS Handler: Bloque 'finally'. Desregistrando cliente.")
        await unregister_client(websocket)

# --- Lógica de Fuente de Datos (Serial o Simulación - sin cambios significativos) ---
async def read_from_serial_port(serial_instance):
    print("MODO SERIAL: Iniciando lector de puerto serie...")
    while True:
        try:
            if not serial_instance.is_open:
                print(f"MODO SERIAL: Puerto serie {serial_instance.port} no abierto. Intentando (re)conectar...")
                try:
                    serial_instance.open()
                    print(f"MODO SERIAL: Puerto {serial_instance.port} (re)conectado.")
                except serial.SerialException as recon_e:
                    print(f"MODO SERIAL: Fallo (re)conectando a {serial_instance.port}: {recon_e}. Reintentando en 5s.")
                    await asyncio.sleep(5)
                    continue
                except Exception as e_open:
                    print(f"MODO SERIAL: Error inesperado (re)abriendo {serial_instance.port}: {e_open}. Reintentando en 5s.")
                    traceback.print_exc()
                    await asyncio.sleep(5)
                    continue
            
            line_bytes = serial_instance.readline()
            if line_bytes:
                try:
                    decoded_line = line_bytes.decode('utf-8').strip()
                    if not decoded_line: continue
                    
                    # Parseamos y guardamos datos en MongoDB
                    try: 
                        data_dict = json.loads(decoded_line)
                        await save_to_mongodb(data_dict)
                    except json.JSONDecodeError:
                        print(f"Error parseando JSON: {decoded_line}")
                                            
                    # Enviamos a todos los clientes WebSocket
                    await send_to_all_clients(decoded_line)

                except UnicodeDecodeError:
                    print(f"Error de decodificación Unicode: {line_bytes}. Saltando.")
                except Exception as e:
                    print(f"Error procesando línea serial: {e}")
                    traceback.print_exc()
            else:
                await asyncio.sleep(0.01) # Pequeña pausa si readline() alcanza timeout
        except serial.SerialException as se:
            print(f"MODO SERIAL: Error de puerto serie: {se}.")
            if serial_instance.is_open:
                try: serial_instance.close()
                except: pass # Ignorar error al cerrar si ya hay problemas
            await asyncio.sleep(5)
        except Exception as e:
            print(f"MODO SERIAL: Error inesperado en lector serial: {e}")
            traceback.print_exc()
            await asyncio.sleep(1)

def get_simulated_sensor_values():
    # Coordenadas base de Valparaíso
    valparaiso_lat_base = -33.0472
    valparaiso_lon_base = -71.6127
    
    # Generar valores simulados que coincidan con tu estructura JSON real
    return {
        # Grupo 1: Inerciales (a) - Acelerómetro (en g)
        "a": [
            round(random.uniform(-2.0, 2.0), 5),  # ax
            round(random.uniform(-2.0, 2.0), 5),  # ay
            round(random.uniform(8.8, 10.8), 5)    # az (incluye gravedad)
        ],
        
        # Grupo 2: Orientación (o) - Ángulos en grados
        "o": [
            round(random.uniform(-180, 180), 2),   # roll
            round(random.uniform(-90, 90), 2),     # pitch
            round(random.uniform(0, 360), 2)       # yaw
        ],
        
        # Grupo 3: Posición (p) - GPS
        "p": [
            round(valparaiso_lat_base + random.uniform(-0.0005, 0.0005), 6),  # lat
            round(valparaiso_lon_base + random.uniform(-0.0005, 0.0005), 6),  # lon
            round(random.uniform(0, 3.0), 2)                                  # speed (km/h)
        ],
        
        # Grupo 4: Ambiente (e) - BME280 + SHT31
        "e": [
            round(random.uniform(10.0, 35.0), 2),  # bme_temp
            round(random.uniform(30.0, 90.0), 2),   # bme_hum
            round(random.uniform(980.0, 1030.0), 2), # bme_pres
            round(random.uniform(10.0, 35.0), 2),   # air_temp (SHT31)
            round(random.uniform(30.0, 90.0), 2)    # air_hum (SHT31)
        ],
        
        # Grupo 5: Agua (w) - Sensores acuáticos + viento
        "w": [
            round(random.uniform(0.0, 25.0), 2),    # wind_mps
            random.randint(0, 359),                 # wind_dir_deg
            round(random.uniform(5.0, 10.0), 2),    # do_mgl
            random.randint(300, 1200),              # tds_ppm
            round(random.uniform(10.0, 20.0), 2),   # water_temp
            round(random.uniform(6.5, 8.5), 2)      # ph
        ],
        
        # Campos adicionales (opcionales para debug)
        "meta": {
            "simulation": True,
            "timestamp": datetime.datetime.utcnow().isoformat()
        }
    }

async def simulate_data_periodically():
    print(f"MODO SIMULACIÓN: Iniciando envío de datos simulados cada {SIMULATION_INTERVAL} segundos.")
    while True:
        try:
            simulated_data = get_simulated_sensor_values()
            json_payload = json.dumps(simulated_data)
            
            # Guardamos datos en mongoDB
            await save_to_mongodb(simulated_data)
            
            await send_to_all_clients(json_payload)
            await asyncio.sleep(SIMULATION_INTERVAL)
        except Exception as e:
            print(f"MODO SIMULACIÓN: Error durante simulación: {e}")
            traceback.print_exc()
            await asyncio.sleep(SIMULATION_INTERVAL)

# --- Lógica del Servidor HTTP (NUEVO/MODIFICADO con aiohttp) ---
BASE_DIR = Path(__file__).resolve().parent # Directorio donde se encuentra este script

async def handle_html_route(request): # Cambiado el nombre para evitar conflicto si tuvieras otra 'handle_html'
    """Sirve el archivo HTML principal."""
    html_file_path = BASE_DIR / HTML_FILENAME
    print(f"HTTP Server: Sirviendo archivo: {html_file_path}")
    try:
        return web.FileResponse(html_file_path)
    except FileNotFoundError:
        print(f"HTTP Server: Archivo '{HTML_FILENAME}' no encontrado en '{BASE_DIR}'.")
        return web.Response(text=f"Error 404: Archivo '{HTML_FILENAME}' no encontrado.", status=404)
    except Exception as e:
        print(f"HTTP Server: Error sirviendo HTML: {e}")
        traceback.print_exc()
        return web.Response(text="Error interno del servidor al servir la página.", status=500)

# --- Configuración de MongoDB ---
async def save_to_mongodb(data_dict):
    """
    Guarda los datos en MongoDB y añade un timestamp
    """
    try:
        # Añadir timestamp de inserción
        data_with_timestamp = {
            **data_dict,
            "timestamp": datetime.utcnow()
        }
        
        result = await db[MONGODB_COLLECTION_NAME].insert_one(data_with_timestamp)
        print(f"Dato guardado en MongoDB, ID: {result.inserted_id}")
        return True
    except Exception as e:
        print(f"Error al guardar en MongoDB: {e}")
        traceback.print_exc()
        return False

# --- Función Principal Modificada ---
async def main():
    global mongo_client, db
    # Configurar aplicación aiohttp para el servidor HTTP
    http_app = web.Application()
    http_app.router.add_get('/', handle_html_route) # Servir HTML_FILENAME en la ruta raíz '/'
    # Si tuvieras archivos CSS/JS/imágenes en una carpeta 'static', podrías añadir:
    # static_dir = BASE_DIR / 'static'
    # if static_dir.is_dir():
    #    http_app.router.add_static('/static/', path=static_dir, name='static')
    #    print(f"HTTP Server: Sirviendo archivos estáticos desde: {static_dir}")

    http_runner = web.AppRunner(http_app)
    await http_runner.setup()
    http_site = web.TCPSite(http_runner, HTTP_HOST, HTTP_PORT)
    
    data_source_task = None
    serial_connection_instance = None
    websocket_server_instance = None # Para poder cerrarlo en finally

    try:
        # Iniciar mongoDB
        print(f"Conectando a MongoDB en {MONGODB_URI}...")
        mongo_client = AsyncIOMotorClient(MONGODB_URI)
        db = mongo_client[MONGODB_DB_NAME]
        print(f"Conectado a MongoDB. Base de datos: {MONGODB_DB_NAME}, Colección: {MONGODB_COLLECTION_NAME}")
        
        # Iniciar servidor HTTP
        await http_site.start()
        print(f"Servidor HTTP iniciado. Accede en: http://{HTTP_HOST}:{HTTP_PORT}")
        if HTTP_HOST == '0.0.0.0':
            # Intenta obtener una IP local para mostrar al usuario (puede no ser siempre la correcta)
            try:
                import socket as s
                hostname = s.gethostname()
                local_ip = s.gethostbyname(hostname)
                print(f"  (Accesible en tu red local en: http://{local_ip}:{HTTP_PORT})")
            except Exception:
                print("  (Si HTTP_HOST es 0.0.0.0, accede desde otra PC en tu red usando la IP de esta máquina)")


        # Iniciar servidor WebSocket
        websocket_server_instance = await websockets.serve(
            websocket_connection_handler,
            WEBSOCKET_HOST,
            WEBSOCKET_PORT
        )
        print(f"Servidor WebSocket iniciado en ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")

        # Iniciar la tarea de origen de datos (serial o simulación)
        if USE_SIMULATED_DATA:
            print("MODO SIMULACIÓN ACTIVADO.")
            data_source_task = asyncio.create_task(simulate_data_periodically())
        else:
            print(f"MODO SERIAL ACTIVADO. Puerto: {SERIAL_PORT}, Baud: {BAUD_RATE}")
            serial_connection_instance = serial.Serial(port=None, baudrate=BAUD_RATE, timeout=SERIAL_TIMEOUT)
            serial_connection_instance.port = SERIAL_PORT
            data_source_task = asyncio.create_task(read_from_serial_port(serial_connection_instance))

        print(f"--- {APP_NAME} Iniciado --- (Presiona Ctrl+C para detener)")
        await asyncio.Event().wait() # Esperar indefinidamente

    except KeyboardInterrupt:
        print(f"\n--- Cerrando {APP_NAME} por interrupción del usuario... ---")
    except Exception as e_main:
        print(f"Error fatal en main(): {e_main}")
        traceback.print_exc()
    finally:
        print("Iniciando proceso de cierre de servicios...")
        
        # Detener tarea de origen de datos
        if data_source_task and not data_source_task.done():
            print("Cancelando tarea de origen de datos (serial/simulación)...")
            data_source_task.cancel()
            try:
                await data_source_task
            except asyncio.CancelledError:
                print("Tarea de origen de datos cancelada.")
            except Exception as e_task_cleanup:
                print(f"Error durante limpieza de tarea de datos: {e_task_cleanup}")
        
        # Cerrar conexión serial si está en modo serial y abierta
        if not USE_SIMULATED_DATA and serial_connection_instance and serial_connection_instance.is_open:
            print("Cerrando puerto serie...")
            try:
                serial_connection_instance.close()
                print("Puerto serie cerrado.")
            except Exception as e_close_serial:
                print(f"Error al cerrar puerto serie: {e_close_serial}")
        
        # Cerrar servidor WebSocket
        if websocket_server_instance:
            print("Cerrando servidor WebSocket...")
            websocket_server_instance.close()
            try:
                await websocket_server_instance.wait_closed()
                print("Servidor WebSocket detenido.")
            except Exception as e_ws_close:
                 print(f"Error al cerrar servidor WebSocket: {e_ws_close}")


        # Detener y limpiar el servidor HTTP aiohttp
        print("Cerrando servidor HTTP...")
        await http_runner.cleanup() # Esto detiene el http_site también
        print("Servidor HTTP detenido.")
        print(f"--- {APP_NAME} Detenido Completamente ---")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # El bloque finally en main() debería manejar la limpieza.
        # Este print es por si el Ctrl+C ocurre muy temprano.
        print("\nPrograma interrumpido.")