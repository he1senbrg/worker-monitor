import asyncio
import websockets
import json
import logging
import sqlite3
import datetime
import sys
from aiohttp import web
import os
from ahrs.filters import Madgwick
import numpy as np

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class SensorDataServer:
    def __init__(self, host="0.0.0.0", port=8765):
        self.host = host
        self.port = port
        self.connected_clients = set()
        self.db_path = "sensor_data.db"
        self.setup_database()
        self.madgwick_filter = Madgwick(sample_period=1 / 100.0)
        self.Q = np.array([1.0, 0.0, 0.0, 0.0])

    def setup_database(self):
        """Initialize SQLite database for storing sensor data"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                device_timestamp INTEGER,
                roll REAL,
                pitch REAL,
                yaw REAL,
                acc_x REAL,
                acc_y REAL,
                acc_z REAL,
                resultant_g REAL,
                gyro_x REAL,
                gyro_y REAL,
                gyro_z REAL,
                mpu_temp REAL,
                bmp_temp REAL,
                pressure REAL,
                altitude REAL,
                dht_temp REAL,
                humidity REAL,
                heat_index TEXT,
                ir_value INTEGER,
                heart_rate REAL,
                avg_heart_rate INTEGER,
                finger_detected BOOLEAN
            )
        """
        )

        conn.commit()
        conn.close()
        logger.info(f"Database initialized: {self.db_path}")

    def store_sensor_data(self, data):
        """Store sensor data in SQLite database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute(
                """
                INSERT INTO sensor_readings (
                    device_timestamp, roll, pitch, yaw, acc_x, acc_y, acc_z, resultant_g,
                    gyro_x, gyro_y, gyro_z, mpu_temp, bmp_temp, pressure,
                    altitude, dht_temp, humidity, heat_index, ir_value,
                    heart_rate, avg_heart_rate, finger_detected
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
                (
                    data.get("timestamp", 0),
                    float(data.get("roll", 0)),
                    float(data.get("pitch", 0)),
                    float(data.get("yaw", 0)),
                    float(data.get("accX", 0)),
                    float(data.get("accY", 0)),
                    float(data.get("accZ", 0)),
                    float(data.get("resultantG", 0)),
                    float(data.get("gyroX", 0)),
                    float(data.get("gyroY", 0)),
                    float(data.get("gyroZ", 0)),
                    float(data.get("mpuTemp", 0)),
                    (
                        float(data.get("bmpTemp", 0))
                        if data.get("bmpTemp") != "Error"
                        else None
                    ),
                    (
                        float(data.get("pressure", 0))
                        if data.get("pressure") != "Error"
                        else None
                    ),
                    (
                        float(data.get("altitude", 0))
                        if data.get("altitude") != "Error"
                        else None
                    ),
                    (
                        float(data.get("dhtTemp", 0))
                        if data.get("dhtTemp") != "Error"
                        else None
                    ),
                    (
                        float(data.get("humidity", 0))
                        if data.get("humidity") != "Error"
                        else None
                    ),
                    data.get("heatIndex", "N/A"),
                    int(data.get("irValue", 0)),
                    float(data.get("heartRate", 0)),
                    int(data.get("avgHeartRate", 0)),
                    data.get("fingerDetected", "false") == "true",
                ),
            )

            conn.commit()
            conn.close()

        except Exception as e:
            logger.error(f"Database error: {e}")

    def process_orientation(self):
        w, x, y, z = self.Q

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.pi / 2 * np.sign(sinp) if np.abs(sinp) >= 1 else np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        return {
            "roll": round(roll_deg, 2),
            "pitch": round(pitch_deg, 2),
            "yaw": round(yaw_deg, 2),
        }

    def process_sensor_data(self, raw_data):
        """Process and display received sensor data"""
        try:
            data = json.loads(raw_data)

            gyro = (
                float(data.get("gyroX", 0)),
                float(data.get("gyroY", 0)),
                float(data.get("gyroZ", 0)),
            )
            acc = (
                float(data.get("accX", 0)),
                float(data.get("accY", 0)),
                float(data.get("accZ", 0)),
            )
            gyro = np.radians(gyro)
            acc = np.array(acc)

            self.Q = self.madgwick_filter.updateIMU(q=self.Q, gyr=gyro, acc=acc)

            orientation_data = self.process_orientation()
            data["roll"] = orientation_data["roll"]
            data["pitch"] = orientation_data["pitch"]
            data["yaw"] = orientation_data["yaw"]
            data["resultantG"] = np.sqrt(
                float(data.get("accX", 0)) ** 2
                + float(data.get("accY", 0)) ** 2
                + float(data.get("accZ", 0)) ** 2
            )

            self.store_sensor_data(data)

        except json.JSONDecodeError as e:
            logger.error(f"JSON decode error: {e}")
        except Exception as e:
            logger.error(f"Data processing error: {e}")

    async def handle_client(self, websocket):
        """Handle individual WebSocket client connections"""
        client_ip = websocket.remote_address[0]
        logger.info(f"New client connected: {client_ip}")

        self.connected_clients.add(websocket)

        try:
            await websocket.send(
                json.dumps(
                    {
                        "type": "welcome",
                        "message": "Connected to sensor data server",
                        "server_time": datetime.datetime.now().isoformat(),
                    }
                )
            )

            async for message in websocket:
                logger.debug(f"Received data from {client_ip}: {len(message)} bytes")
                self.process_sensor_data(message)

                await websocket.send(
                    json.dumps(
                        {
                            "type": "ack",
                            "status": "received",
                            "timestamp": datetime.datetime.now().isoformat(),
                        }
                    )
                )

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client {client_ip} disconnected")
        except Exception as e:
            logger.error(f"Error handling client {client_ip}: {e}")
        finally:
            self.connected_clients.discard(websocket)

    async def start_server(self):
        """Start the WebSocket server"""
        logger.info(f"Starting WebSocket server on {self.host}:{self.port}")

        server = await websockets.serve(
            self.handle_client, self.host, self.port, ping_interval=20, ping_timeout=10
        )

        logger.info("WebSocket server started successfully!")
        logger.info(f"Listening for ESP32 connections on ws://{self.host}:{self.port}")
        logger.info("Press Ctrl+C to stop the server")

        return server

    def get_recent_data(self, limit=10):
        """Get recent sensor readings from database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT * FROM sensor_readings 
                ORDER BY timestamp DESC 
                LIMIT ?
            """,
                (limit,),
            )

            rows = cursor.fetchall()
            columns = [description[0] for description in cursor.description]

            conn.close()

            return [dict(zip(columns, row)) for row in rows]

        except Exception as e:
            logger.error(f"Error retrieving data: {e}")
            return []

    def cleanup(self):
        """Cleanup function for graceful shutdown"""
        logger.info("Shutting down server...")

        for client in self.connected_clients:
            asyncio.create_task(client.close())


def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info("Received shutdown signal")
    sys.exit(0)


sensor_server = SensorDataServer()


async def index(request):
    if not os.path.exists("./static"):
        os.makedirs("./static")
    return web.FileResponse("./static/index.html")


async def get_recent_data(request):
    """Return recent sensor data for visualizations."""
    limit = int(request.query.get("limit", 50))
    data = sensor_server.get_recent_data(limit=limit)

    for item in data:
        if "timestamp" in item and isinstance(item["timestamp"], str):
            pass
        elif "timestamp" in item:
            item["timestamp"] = (
                item["timestamp"].isoformat()
                if hasattr(item["timestamp"], "isoformat")
                else str(item["timestamp"])
            )

    return web.json_response(data)


async def start_servers():
    ws_server = await sensor_server.start_server()

    app = web.Application()

    app.add_routes(
        [
            web.get("/", index),
            web.get("/data", get_recent_data),
            web.static("/static", "./static"),
        ]
    )

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8080)
    await site.start()

    logger.info("HTTP server started on http://0.0.0.0:8080")
    logger.info("Dashboard available at http://0.0.0.0:8080/")
    await ws_server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(start_servers())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
