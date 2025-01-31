import json
import time
import serial
import logging as log
from typing import Dict, Any, Optional
from threading import Thread, Event

log.basicConfig(level=log.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class CommunicationManager:
    def __init__(self, port: str = 'COM7', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.message_end = b'\n'

        self.serial_port = None
        self.is_connected = False
        self._read_thread = None

        self.buffer = bytearray()
        self._stop_event = Event()

    def connect(self) -> bool:
        """serial connection"""
        if self.is_connected:
            return True

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=10,
                write_timeout=10
            )
            self.is_connected = True

            # read loop
            self._stop_event.clear()
            self._read_thread = Thread(target=self._read_loop)
            self._read_thread.daemon = True
            self._read_thread.start()
            return True

        except Exception as e:
            log.error(f'Error connecting to serial port: {str(e)}')
            return False

    def send_message(self, message_type: str, data: dict) -> bool:
        """
        send json message to robot
        message_type: type message ('command', 'status', etc)
        data: dict with data
        """
        if not self.is_connected or not self.serial_port:
            log.error("Error: Puerto serial no inicializado")
            return False

        try:
            message = {
                'type': message_type,
                'data': data,
            }
            encoded_message = json.dumps(message).encode() + self.message_end
            log.info(f'send message: {encoded_message}')
            self.serial_port.write(encoded_message)
            return True
        except Exception as e:
            print(f"Error enviando mensaje: {e}")
            return False

    def _read_loop(self):
        """Thread loop for read messages from VEX"""
        while not self._stop_event.is_set():
            if self.serial_port and self.serial_port.in_waiting:
                try:
                    char = self.serial_port.read()
                    if char == self.message_end:
                        message = self.buffer.decode()
                        self.buffer = bytearray()
                        try:
                            data = json.loads(message)
                            self._process_message(data)
                        except json.JSONDecodeError:
                            log.error(f'error message decode: {message}')
                    else:
                        self.buffer.extend(char)
                except Exception as e:
                    log.error(f'error read serial port: {e}')
            time.sleep(0.01)

    def _process_message(self, message: dict):
        """process message from VEX"""
        try:
            msg_type = message.get('type', '').lower()
            data = message.get('data', {})

            if msg_type == 'check_service':
                state = data.get('state')
                log.info(f'{msg_type} status:\nstate: {state}')

            elif msg_type == 'safety_service':
                state = data.get('state')
                time_taken = data.get('time')
                log.info(f"{msg_type} status: \nstate: {state}, \ntime: {time_taken}s")
                if state == 'error':
                    log.error(f"Safety Service Error: {data.get('error_msg', 'Unknown error')}")

            elif msg_type == 'scan_service':
                state = data.get('state')
                if state == 'in_progress':
                    log.info(f"Scan Data - Object Detected: {data['object']}, "
                             f"Angle: {data['center_angle']}°, "
                             f"Distance: {data['distance']}mm, ")

                elif state == 'complete':
                    objects = data.get('objects', [])
                    log.info(f"Scan Complete - Objects detected: {len(objects)}")
                    for obj in objects:
                        log.info(f"Object at {obj['center_angle']}°, "
                                 f"width: {obj['width']}mm, "
                                 f"distance: {obj['distance']}mm, "
                                 f"size: {obj['max_size']}mm")
        except Exception as e:
            log.error(f'error process message: {e}')

    def close(self):
        """"close serial connection"""
        self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            log.info('Serial connection closed')


# Ejemplo de uso
if __name__ == "__main__":
    serial_manager = CommunicationManager()
    try:
        if serial_manager.connect():
            log.info("Conectado al VEX Brain")

            # Ejemplo de envío de comandos
            message_type = 'scan_service'
            data = {'data': 'None'}
            serial_manager.send_message(message_type, data)
            #time.sleep(2)
            #serial_manager.send_command('safety')
            #time.sleep(5)
            #serial_manager.send_command('scan')

            # Mantener el programa corriendo para recibir mensajes
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        log.info("Programa terminado por el usuario")
    finally:
        serial_manager.close()
