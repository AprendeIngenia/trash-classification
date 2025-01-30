import json
import time
import serial
import logging as log
log.basicConfig(level=log.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class SerialCommunication:
    def __init__(self):
        self.com = serial.Serial("COM7", 115200, write_timeout=10)
        self.buffer = bytearray()
        self.message_end = b'\n'

    def sending_data(self, command: str) -> None:
        self.com.write(command.encode('ascii'))
        print(f'SENDING DATA: {command}')

    def send_message(self, message_type: str, data: dict) -> bool:
        """
        Envía un mensaje JSON al robot
        message_type: tipo de mensaje ('command', 'status', etc)
        data: diccionario con los datos del mensaje
        """
        try:
            message = {
                'type': message_type,
                'data': data,
                'timestamp': time.time()
            }
            encoded_message = json.dumps(message).encode() + self.message_end
            self.com.write(encoded_message)
            return True
        except Exception as e:
            print(f"Error enviando mensaje: {e}")
            return False

    def send_command(self, command: str, params: dict = {}):
        """
        Envía un comando al robot
        command: comando a enviar ('check', 'safety', 'scan', 'stop')
        params: parámetros adicionales del comando (opcional)
        """
        data = {'command': command}
        if params:
            data.update(params)
        return self.send_message('command', data)


class SendExample:
    def __init__(self):
        self.serial_manager = SerialCommunication()

    def run(self):
        try:
            self.serial_manager.send_command('scan')

            time.sleep(0.1)
        except KeyboardInterrupt:
            log.info('\n closing serial connection')

        finally:
            self.serial_manager.com.close()
            log.info('Serial connection closed')


if __name__ == '__main__':
    robot = SendExample()
    robot.run()
