#!/usr/bin/env python3
"""
ESP32 OTA Update Tool via MQTT
This script handles firmware updates for ESP32 devices over MQTT
"""

import paho.mqtt.client as mqtt
import time
import os
import sys
import argparse
import hashlib

# MQTT Settings
MQTT_BROKER = "mqttiot.loranet.my"
MQTT_PORT = 1885
MQTT_USER = "iotdbuser"
MQTT_PASSWORD = "IoTdb2024"
CHUNK_SIZE = 1024  # Size of each firmware chunk
MAX_RETRIES = 3    # Maximum number of retries for failed chunks
DEFAULT_TIMEOUT = 60  # Default timeout in seconds

# MQTT Topics (will be formatted with device_id)
MQTT_TOPICS = {
    'ota_command': "ble-scanner/{device_id}/ota/command",
    'ota_data': "ble-scanner/{device_id}/ota/data",
    'ota_status': "ble-scanner/{device_id}/ota/status",
    'ota_progress': "ble-scanner/{device_id}/ota/progress"
}

class OTAUpdater:
    def __init__(self, broker, port, username, password, device_id="default", verbose=False):
        self.client = mqtt.Client()
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.broker = broker
        self.port = port
        self.device_id = device_id
        self.verbose = verbose
        self.connected = False
        self.update_complete = False
        self.progress = 0
        self.failed_chunks = set()
        self.current_chunk = 0
        self.total_chunks = 0
        
        # Initialize topics for this device
        self.topics = {k: v.format(device_id=device_id) for k, v in MQTT_TOPICS.items()}

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"Connected to MQTT Broker: {self.broker}")
            self.connected = True
            # Subscribe to device-specific topics
            client.subscribe(self.topics['ota_status'])
            client.subscribe(self.topics['ota_progress'])
            if self.verbose:
                print(f"Subscribed to topics: {self.topics['ota_status']}, {self.topics['ota_progress']}")
        else:
            print(f"Failed to connect, return code: {rc}")

    def on_message(self, client, userdata, msg):
        try:
            message = msg.payload.decode()
            if msg.topic == self.topics['ota_progress']:
                try:
                    self.progress = int(message)
                    self.print_progress()
                except ValueError:
                    if self.verbose:
                        print(f"Invalid progress value received: {message}")
            elif msg.topic == self.topics['ota_status']:
                if "chunk_failed" in message:
                    chunk_num = int(message.split(":")[1])
                    self.failed_chunks.add(chunk_num)
                    if self.verbose:
                        print(f"\nChunk {chunk_num} failed, will retry")
                elif "successful" in message:
                    print("\nUpdate successful!")
                    self.update_complete = True
                elif self.verbose:
                    print(f"\nStatus message received: {message}")
        except Exception as e:
            if self.verbose:
                print(f"\nError processing message: {e}")

    def print_progress(self):
        status = f"Progress: {self.progress}% [{self.current_chunk}/{self.total_chunks} chunks]"
        if self.failed_chunks:
            status += f" (Retrying {len(self.failed_chunks)} chunks)"
        sys.stdout.write(f"\r{status}")
        sys.stdout.flush()

    def connect(self):
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        timeout = 5
        while not self.connected and timeout > 0:
            time.sleep(1)
            timeout -= 1
        return self.connected

    def calculate_checksum(self, firmware_path):
        """Calculate MD5 checksum of firmware file"""
        md5_hash = hashlib.md5()
        with open(firmware_path, "rb") as f:
            for byte_block in iter(lambda: f.read(4096), b""):
                md5_hash.update(byte_block)
        return md5_hash.hexdigest()

    def send_firmware(self, firmware_path, chunk_size=CHUNK_SIZE, timeout=DEFAULT_TIMEOUT):
        if not os.path.exists(firmware_path):
            print(f"Error: Firmware file not found: {firmware_path}")
            return False

        # Get firmware size and checksum
        firmware_size = os.path.getsize(firmware_path)
        checksum = self.calculate_checksum(firmware_path)
        
        print(f"Firmware size: {firmware_size} bytes")
        print(f"MD5 Checksum: {checksum}")
        
        # Calculate total chunks for progress reporting
        self.total_chunks = (firmware_size + chunk_size - 1) // chunk_size
        
        # Send start message with size and checksum
        start_message = f"START:{firmware_size}:{checksum}"
        self.client.publish(self.topics['ota_command'], start_message)
        if self.verbose:
            print(f"Sent start message: {start_message}")
        time.sleep(1)

        retry_count = 0
        while retry_count < MAX_RETRIES:
            # Read and send firmware in chunks
            with open(firmware_path, "rb") as file:
                self.current_chunk = 0
                while self.current_chunk < self.total_chunks:
                    # Skip chunks that were successfully transmitted
                    if self.current_chunk not in self.failed_chunks:
                        chunk = file.read(chunk_size)
                        if not chunk:
                            break
                        # Send chunk with its sequence number
                        message = f"{self.current_chunk}:{chunk.hex()}"
                        self.client.publish(self.topics['ota_data'], message)
                        if self.verbose:
                            print(f"\nSent chunk {self.current_chunk}/{self.total_chunks}")
                        time.sleep(0.1)  # Small delay to prevent flooding
                    self.current_chunk += 1

            # If no chunks failed, break the retry loop
            if not self.failed_chunks:
                break
            
            retry_count += 1
            if retry_count < MAX_RETRIES and self.failed_chunks:
                print(f"\nRetrying {len(self.failed_chunks)} failed chunks (attempt {retry_count}/{MAX_RETRIES})")
                self.current_chunk = 0  # Reset for progress display

        # Send end message
        self.client.publish(self.topics['ota_command'], "END")
        if self.verbose:
            print("\nSent END message")
        
        # Wait for completion or timeout
        remaining_timeout = timeout
        while not self.update_complete and remaining_timeout > 0:
            time.sleep(1)
            remaining_timeout -= 1
            if self.verbose and remaining_timeout % 10 == 0:
                print(f"\nWaiting for completion... {remaining_timeout}s remaining")

        if not self.update_complete and self.verbose:
            print(f"\nUpdate timed out after {timeout} seconds")

        return self.update_complete

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

def find_firmware():
    """Find firmware.bin in PlatformIO build directory"""
    pio_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                           '.pio', 'build', 'esp32dev', 'firmware.bin')
    if os.path.exists(pio_path):
        return pio_path
    return None

def main():
    parser = argparse.ArgumentParser(description='ESP32 OTA Update Tool')
    parser.add_argument('firmware', nargs='?', help='Path to firmware.bin file')
    parser.add_argument('--broker', default=MQTT_BROKER, help='MQTT broker address')
    parser.add_argument('--port', type=int, default=MQTT_PORT, help='MQTT broker port')
    parser.add_argument('--user', default=MQTT_USER, help='MQTT username')
    parser.add_argument('--password', default=MQTT_PASSWORD, help='MQTT password')
    parser.add_argument('--device-id', default='default', help='Target device ID')
    parser.add_argument('--timeout', type=int, default=DEFAULT_TIMEOUT, 
                       help='Timeout in seconds for update completion')
    parser.add_argument('--verbose', action='store_true', 
                       help='Enable verbose output')
    args = parser.parse_args()
    
    # If no firmware path provided, try to find it in PlatformIO build directory
    if not args.firmware:
        firmware_path = find_firmware()
        if firmware_path:
            print(f"Using firmware from PlatformIO build: {firmware_path}")
            args.firmware = firmware_path
        else:
            print("Error: No firmware file specified and couldn't find one in PlatformIO build directory")
            sys.exit(1)

    updater = OTAUpdater(
        broker=args.broker,
        port=args.port,
        username=args.user,
        password=args.password,
        device_id=args.device_id,
        verbose=args.verbose
    )
    
    try:
        if updater.connect():
            if args.verbose:
                print(f"Starting OTA update for device: {args.device_id}")
            if updater.send_firmware(args.firmware, timeout=args.timeout):
                print("\nOTA update completed successfully!")
            else:
                print("\nOTA update failed or timed out!")
        else:
            print("Failed to connect to MQTT broker!")
    except KeyboardInterrupt:
        print("\nUpdate cancelled by user")
    finally:
        updater.disconnect()

if __name__ == "__main__":
    main()
