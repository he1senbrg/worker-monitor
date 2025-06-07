#!/usr/bin/env python3
"""
Multi-Sensor WebSocket Server
Receives real-time sensor data from ESP32 via WebSocket
"""

import asyncio
import websockets
import json
import logging
import sqlite3
import datetime
import signal
import sys
from aiohttp import web
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SensorDataServer:
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.connected_clients = set()
        self.db_path = "sensor_data.db"
        self.setup_database()
        
    def setup_database(self):
        """Initialize SQLite database for storing sensor data"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                device_timestamp INTEGER,
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
        ''')
        
        conn.commit()
        conn.close()
        logger.info(f"Database initialized: {self.db_path}")
    
    def store_sensor_data(self, data):
        """Store sensor data in SQLite database"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO sensor_readings (
                    device_timestamp, acc_x, acc_y, acc_z, resultant_g,
                    gyro_x, gyro_y, gyro_z, mpu_temp, bmp_temp, pressure,
                    altitude, dht_temp, humidity, heat_index, ir_value,
                    heart_rate, avg_heart_rate, finger_detected
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                data.get('timestamp', 0),
                float(data.get('accX', 0)),
                float(data.get('accY', 0)),
                float(data.get('accZ', 0)),
                float(data.get('resultantG', 0)),
                float(data.get('gyroX', 0)),
                float(data.get('gyroY', 0)),
                float(data.get('gyroZ', 0)),
                float(data.get('mpuTemp', 0)),
                float(data.get('bmpTemp', 0)) if data.get('bmpTemp') != 'Error' else None,
                float(data.get('pressure', 0)) if data.get('pressure') != 'Error' else None,
                float(data.get('altitude', 0)) if data.get('altitude') != 'Error' else None,
                float(data.get('dhtTemp', 0)) if data.get('dhtTemp') != 'Error' else None,
                float(data.get('humidity', 0)) if data.get('humidity') != 'Error' else None,
                data.get('heatIndex', 'N/A'),
                int(data.get('irValue', 0)),
                float(data.get('heartRate', 0)),
                int(data.get('avgHeartRate', 0)),
                data.get('fingerDetected', 'false') == 'true'
            ))
            
            conn.commit()
            conn.close()
            
        except Exception as e:
            logger.error(f"Database error: {e}")
    
    def process_sensor_data(self, raw_data):
        """Process and display received sensor data"""
        try:
            data = json.loads(raw_data)
            
            # Store in database
            self.store_sensor_data(data)
            
            # Display formatted data
            print("\n" + "="*80)
            print(f"SENSOR DATA RECEIVED - {datetime.datetime.now().strftime('%H:%M:%S')}")
            print("="*80)
            
            # IMU Data
            print("📐 ACCELEROMETER & GYROSCOPE (MPU6500) - KALMAN FILTERED")
            print("-"*60)
            print(f"Acceleration (g): X={data.get('accX', 'N/A'):>8} | Y={data.get('accY', 'N/A'):>8} | Z={data.get('accZ', 'N/A'):>8}")
            print(f"Resultant G-Force: {data.get('resultantG', 'N/A'):>8} g")
            print(f"Gyroscope (°/s):  X={data.get('gyroX', 'N/A'):>8} | Y={data.get('gyroY', 'N/A'):>8} | Z={data.get('gyroZ', 'N/A'):>8}")
            print(f"IMU Temperature:  {data.get('mpuTemp', 'N/A'):>8} °C")
            
            # Environmental Data
            print("\n🌡️ ENVIRONMENTAL SENSORS")
            print("-"*60)
            print(f"BMP Temperature:  {data.get('bmpTemp', 'N/A'):>8} °C")
            print(f"Pressure:         {data.get('pressure', 'N/A'):>8} hPa")
            print(f"Altitude:         {data.get('altitude', 'N/A'):>8} m")
            print(f"DHT Temperature:  {data.get('dhtTemp', 'N/A'):>8} °C")
            print(f"Humidity:         {data.get('humidity', 'N/A'):>8} %")
            print(f"Heat Index:       {data.get('heatIndex', 'N/A'):>8}")
            
            # Biometric Data
            print("\n❤️ BIOMETRIC SENSORS")
            print("-"*60)
            print(f"IR Signal:        {data.get('irValue', 'N/A'):>8}")
            finger_status = "✅ Detected" if data.get('fingerDetected') == 'true' else "❌ Not detected"
            print(f"Finger:           {finger_status:>15}")
            if data.get('fingerDetected') == 'true':
                print(f"Heart Rate:       {data.get('heartRate', 'N/A'):>8} BPM")
                print(f"Average BPM:      {data.get('avgHeartRate', 'N/A'):>8}")
            
            print("="*80)
            
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
            # Send welcome message
            await websocket.send(json.dumps({
                "type": "welcome",
                "message": "Connected to sensor data server",
                "server_time": datetime.datetime.now().isoformat()
            }))
            
            async for message in websocket:
                logger.debug(f"Received data from {client_ip}: {len(message)} bytes")
                self.process_sensor_data(message)
                
                # Echo back confirmation (optional)
                await websocket.send(json.dumps({
                    "type": "ack",
                    "status": "received",
                    "timestamp": datetime.datetime.now().isoformat()
                }))
                
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client {client_ip} disconnected")
        except Exception as e:
            logger.error(f"Error handling client {client_ip}: {e}")
        finally:
            self.connected_clients.discard(websocket)
    
    async def start_server(self):
        """Start the WebSocket server"""
        logger.info(f"Starting WebSocket server on {self.host}:{self.port}")
        
        # Start the server
        server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=10
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
            
            cursor.execute('''
                SELECT * FROM sensor_readings 
                ORDER BY timestamp DESC 
                LIMIT ?
            ''', (limit,))
            
            rows = cursor.fetchall()
            columns = [description[0] for description in cursor.description]
            
            conn.close()
            
            # Convert to list of dictionaries
            return [dict(zip(columns, row)) for row in rows]
            
        except Exception as e:
            logger.error(f"Error retrieving data: {e}")
            return []
    
    def cleanup(self):
        """Cleanup function for graceful shutdown"""
        logger.info("Shutting down server...")
        # Close any remaining connections
        for client in self.connected_clients:
            asyncio.create_task(client.close())

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    logger.info("Received shutdown signal")
    sys.exit(0)

# async def main():
#     # Set up signal handlers
#     signal.signal(signal.SIGINT, signal_handler)
#     signal.signal(signal.SIGTERM, signal_handler)
    
#     # Create and start server
#     server_instance = SensorDataServer()
#     server = await server_instance.start_server()
    
#     try:
#         # Keep the server running
#         await server.wait_closed()
#     except KeyboardInterrupt:
#         logger.info("Server shutdown requested")
#     finally:
#         server_instance.cleanup()
#         server.close()
#         await server.wait_closed()

sensor_server = SensorDataServer()

# Modify the index function to serve the index.html file
async def index(request):
    # Make sure the static directory exists
    if not os.path.exists('./static'):
        os.makedirs('./static')
    return web.FileResponse('./static/index.html')

# Modify the get_recent_data function to format data properly for frontend
async def get_recent_data(request):
    """Return recent sensor data for visualizations."""
    limit = int(request.query.get('limit', 50))
    data = sensor_server.get_recent_data(limit=limit)
    
    # Process data to ensure JSON serialization works correctly
    for item in data:
        # Convert datetime objects to ISO format strings
        if 'timestamp' in item and isinstance(item['timestamp'], str):
            # SQLite might return timestamps as strings, keep them as is
            pass
        elif 'timestamp' in item:
            item['timestamp'] = item['timestamp'].isoformat() if hasattr(item['timestamp'], 'isoformat') else str(item['timestamp'])
    
    return web.json_response(data)

async def start_servers():
    # Start WebSocket server
    ws_server = await sensor_server.start_server()

    # Set up aiohttp web app
    app = web.Application()
    
    # Add routes for API and static files
    app.add_routes([
        web.get('/', index),
        web.get('/data', get_recent_data),
        web.static('/static', './static'),  # Serve other static files
    ])

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 8080)
    await site.start()

    logger.info("HTTP server started on http://0.0.0.0:8080")
    logger.info("Dashboard available at http://0.0.0.0:8080/")
    await ws_server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(start_servers())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
