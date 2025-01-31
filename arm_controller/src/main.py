# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       aprende e ingenia                                            #
# 	Created:      12/14/2024, 10:56:51 PM                                      #
# 	Description:  IQ2 project                                                  #
#                                                                              #
# ---------------------------------------------------------------------------- #
#vex:disable=repl

# ============================================================================ #
#                       module imports
# ============================================================================ #
from vex import *
import time
import json

# ============================================================================ #
#                       state management
# ============================================================================ #
class LEDColors:
    ERROR = (255, 0, 0)             # Red: Error/Stop
    WARNING = (255, 150, 0)         # Orange: Warning
    READY = (0, 255, 0)             # Green: Ready
    RUNNING = (0, 0, 255)           # Blue: Process Running
    OBJECT_DETECTED = (0, 255, 155) # Cyan: Object Detected
    
# ============================================================================ #
#                       communication manager                                  
# ============================================================================ #
class CommunicationManager:
    def __init__(self):
        self.serial_port = None
        self.buffer = bytearray()
        self.message_end = b'\n'
        
    def initialize(self):
        try:
            self.serial_port = open('/dev/serial1', 'rb+')
        except:
            raise Exception('serial port not available')
        
    def read_message(self):
        char = self.serial_port.read(1)
        if char == self.message_end:
            message = self.buffer.decode()
            self.buffer = bytearray()
            try:
                return json.loads(message)
            except json.JSONDecodeError:
                return None
        else:
            self.buffer.extend(char)
        return None
        
    def send_message(self, message_type: str, data: dict) -> bool:
        if not self.serial_port:
            return False
        
        message = {
            'type': message_type,
            'data': data,
        }
        encoded_message = json.dumps(message).encode() + self.message_end
        self.serial_port.write(encoded_message)
        return True
        
    def close(self):
        if self.serial_port:
            self.serial_port.close()
            

# ============================================================================ #
#                       sensor module
# ============================================================================ #
class SensorModule:
    def __init__(self):
        self.brain = Brain()
        self.inertial = Inertial()
        self.gripper_distance = Distance(Ports.PORT7)
        self.touchled = Touchled(Ports.PORT10)
        self.base_distance = Distance(Ports.PORT11)
        self.bumper = Bumper(Ports.PORT12)
    
    # screen methods
    def clear_screen(self):
        self.brain.screen.clear_screen()
        
    def print_screen(self, text:str, coordinate_x: int, coordinate_y: int):
        self.brain.screen.print_at(text, x=coordinate_x, y=coordinate_y)
    
    # inertial methods
    def calibrate_intertial_sensor(self):
        self.inertial.calibrate()
        
    def get_angle(self):
        return self.inertial.heading()
    
    # distance methods
    def get_base_distance(self):
        return self.base_distance.object_distance(MM)
    
    def get_base_object_size(self):
        return self.base_distance.object_rawsize()
    
    def get_gripper_distance(self):
        return self.gripper_distance.object_distance(MM)
    
    # bumper methods
    def is_bumper_pressed(self):
        return self.bumper.pressing()
    
    # touchled methods
    def set_color(self, color):
        r, g, b = color
        self.touchled.set_color(r,g,b)
        
    # general methods
    def check_sensors(self):
        return (self.base_distance.installed() and
                self.gripper_distance.installed() and
                self.bumper.installed())
        

# ============================================================================ #
#                       perception module
# ============================================================================ #
class PerceptionModule:
    def __init__(self, sensor_module: SensorModule):
        self.sensor = sensor_module
        self.object_detected = False
        self.current_object_size = 0
        
    def process_sensor_data(self):
        base_distance = self.sensor.get_base_distance()
        if 50 <= base_distance <= 300:
            self.current_object_size = self.sensor.get_base_object_size()
            self.sensor.set_color(LEDColors.OBJECT_DETECTED)
            self.object_detected = True
            
        else:
            self.current_object_size = 0
            self.object_detected = False
            self.sensor.set_color(LEDColors.RUNNING)
            
        return {
            'distance': base_distance,
            'size': self.current_object_size,
            'detected': self.object_detected
        }
        
        
# ============================================================================ #
#                       mapping module
# ============================================================================ #
class MappingModule:
    def __init__(self):
        self.objects_map = []
        self.current_object = {
            'start_angle': 0,
            'end_angle': 0,
            'max_size': 0,
            'distance': 0,
            'is_tracking': False
        }
    
    def process_object_detection(self, angle, object_size, distance):
        if object_size > 0:
            # new object detected
            if not self.current_object['is_tracking']:
                self.current_object['start_angle'] = angle
                self.current_object['is_tracking'] = True
                self.current_object['distance'] = distance
                self.current_object['max_size'] = object_size
            else:
                self.current_object['max_size'] = max(self.current_object['max_size'], object_size)
                
        elif self.current_object['is_tracking']:
            self.current_object['end_angle'] = angle
            self._save_object()
            
    def _save_object(self):
        start = self.current_object['start_angle']
        end = self.current_object['end_angle']
        
        # calculate center angle considering 360° wraparound
        if end < start:
            if (start - end) > 180:
                center = (start + (end + 360)) / 2
                if center >= 360:
                    center -= 360
            else:
                center = (start + end) / 2
                
        else:
            center = (start + end) / 2
            
        self.objects_map.append({
            'center_angle': center,
            'width': abs(end - start),
            'distance': self.current_object['distance'],
            'max_size': self.current_object['max_size']
        })
        
        # reset tracking
        self.current_object['is_tracking'] = False
        self.current_object['max_size'] = 0
        self.current_object['distance'] = 0
        
    def get_objects_map(self):
        objects_detected = self.objects_map
        self.objects_map = []
        return objects_detected
    

# ============================================================================ #
#                       control module
# ============================================================================ #
class ControlModule:
    def __init__(self):
        self.base_motor = Motor(Ports.PORT1, True)
        self.shoulder_motor = Motor(Ports.PORT2, True)
        self.elbow_motor = Motor(Ports.PORT3, True)
        self.gripper_motor = Motor(Ports.PORT4, True)
        
    # rotate motor methods
    def rotate_motor_forward(self, motor, speed):
        motor.spin(FORWARD, speed, RPM)
        
    def rotate_motor_reverse(self, motor, speed):
        motor.spin(REVERSE, speed, RPM)
        
    # current motor methods
    def get_current_motor(self, motor):
        return motor.current()
    
    # general motor methods
    def general_stop(self):
        self.base_motor.stop()
        self.shoulder_motor.stop()
        self.elbow_motor.stop()
        self.gripper_motor.stop()
    
    def check_motors(self):
        return (self.base_motor.installed() and
                self.shoulder_motor.installed() and
                self.elbow_motor.installed() and
                self.gripper_motor.installed())
    

# ============================================================================ #
#                       safety module
# ============================================================================ #
class SafetyModule:
    def __init__(self, sensor_module: SensorModule, control_module: ControlModule):
        self.sensor_module = sensor_module
        self.control_module = control_module
        self.error_count = 0
        self.error_threshold = 3
    
    # check methods
    def check_motors(self):
        return self.control_module.check_motors()
    
    def check_sensors(self):
        return self.sensor_module.check_sensors()
        
    def check_shoulder_safety(self) -> bool:
        if self.sensor_module.is_bumper_pressed():
            self.control_module.general_stop()
            self.sensor_module.set_color(LEDColors.ERROR)
            self.control_module.rotate_motor_reverse(self.control_module.shoulder_motor, 10)
            time.sleep(2)
            self.control_module.general_stop()
            return True
        else:
            self.control_module.rotate_motor_forward(self.control_module.shoulder_motor, 60)
            self.sensor_module.set_color(LEDColors.WARNING)
            return False
        
    def check_gripper_safety(self) -> bool:
        gripper_current = self.control_module.get_current_motor(self.control_module.gripper_motor)
        self.control_module.rotate_motor_forward(self.control_module.gripper_motor, 20)

        if gripper_current > 0.3:
            self.control_module.general_stop()
            return True
        else:
            return False

# ============================================================================ #
#                       main module
# ============================================================================ #
class RoboticServices:
    def __init__(self):
        # initialize modules
        self.sensor_module = SensorModule()
        self.perception_module = PerceptionModule(self.sensor_module)
        self.mapping_module = MappingModule()
        self.control_module = ControlModule()
        self.safety_module = SafetyModule(self.sensor_module, self.control_module)
        self.communication = CommunicationManager()
        
        
        # -- Variables / banderas internas --
        # "check_service"
        self.check_mode_active = False
        
        # "safety_service"
        self.safety_mode_active = False
        self.safety_shoulder = False
        self.safety_gripper = False
        self.safety_service_loop = True
        
        # "scan_service"
        self.scan_mode_active = False
        self.scan_start_time = 0
        self.scan_timeout = 30
        self.accumulated_rotation = 0
        self.last_angle = 0
        self.scan_start = True
        self.scan_update = False
        self.scan_complete = False
        self.scan_error = False
    
        # calibrate inertial sensor
        #self.sensor_module.calibrate_intertial_sensor()
            
    # check service
    def check_service(self) -> bool:
        if self.safety_module.check_sensors() and self.safety_module.check_motors():
            self.sensor_module.set_color(LEDColors.READY)
            return True
        else:
            self.sensor_module.set_color(LEDColors.ERROR)
            return False
    
    # safety state service
    def safety_state_service(self) -> bool:
        """
        Called periodically. Performs:
        - check_shoulder_safety().
        - when that finishes (True), it passes to check_gripper_safety().
        - when gripper finishes, it finishes everything.
        Returns True when the ENTIRE sequence is finished.
        """
        while self.safety_service_loop ==  True:
            if not self.safety_shoulder:
                safety_shoulder = self.safety_module.check_shoulder_safety()
                if safety_shoulder:
                    self.safety_shoulder = True
            else:
                if not self.safety_gripper:
                    safety_gripper = self.safety_module.check_gripper_safety()
                    if safety_gripper:
                        self.safety_gripper = True
                        self.safety_service_loop = False
                        self.safety_mode_active = True
                        self.sensor_module.set_color(LEDColors.READY)
                        return True
        return False
    
    def reset_safety_sequence(self):
        """
        Reset the security sequence variables.
        Call every time you want to restart it..
        """
        self.safety_mode_active = False
        self.safety_shoulder = False
        self.safety_gripper = False
        self.safety_service_loop = True
        
    # scan service
    def scan_service(self):
        while not self.scan_complete:
            if self.scan_start:
                scan_init = self.start_scan_service()
                if scan_init:
                    self.scan_start = False
                    self.scan_update = True
            elif self.scan_update:
                scan_finish = self.update_scan_service()
                if scan_finish:
                    self.scan_update = False
                    self.scan_mode_active = False
                    self.scan_complete = True
                    return True
                    
        return self.scan_complete
                    
    def start_scan_service(self):
        """Initialize scan service"""
        if self.scan_complete:
            return False
        
        self.scan_mode_active = True
        
        self.scan_start_time = time.time()
        self.last_angle = self.sensor_module.get_angle()
        self.control_module.rotate_motor_forward(self.control_module.base_motor, 20)
        self.sensor_module.set_color(LEDColors.RUNNING)
        return True
        
    def update_scan_service(self):
        current_angle = self.sensor_module.get_angle()
        
        # Calculate rotation
        delta = current_angle - self.last_angle
        if delta > 180: delta -= 360
        elif delta < -180: delta += 360
        
        self.accumulated_rotation += abs(delta)
        self.last_angle = current_angle
        
        # Process sensor data
        sensor_data = self.perception_module.process_sensor_data()
        self.mapping_module.process_object_detection(current_angle, sensor_data['size'], sensor_data['distance']) 
                
        # Check if scan is complete
        if self.accumulated_rotation >= 360 or time.time() - self.scan_start_time > self.scan_timeout:
            self.control_module.general_stop()
            return True
        
        return False
            

            
    def reset_scan_sequence(self):
        """
        Reset the scan sequence variables.
        Call every time you want to restart it..
        """
        self.scan_mode_active = False
        self.scan_start_time = 0
        self.scan_timeout = 30
        self.accumulated_rotation = 0
        self.last_angle = 0
        self.scan_start = True
        self.scan_update = False
        self.scan_complete = False
            
    # communication methods
    def process_raspberry_message(self, message:dict):
        """Process JSON messages from Raspberry Pi"""
        if not message or 'type' not in message:
            return
            
        msg_type = message['type'].lower()
        msg_data = message.get('data', {})
            
        if msg_type == 'check_service':
            self.check_mode_active = True
            
        elif msg_type == 'safety_service':
            self.reset_safety_sequence()
            self.safety_mode_active = True
            
        elif msg_type == 'scan_service':
            self.reset_scan_sequence()
            self.scan_mode_active = True
                
            
    # run method
    def run(self):
        self.communication.initialize()
        while True:
            try:
                # process incoming messages
                message = self.communication.read_message()
                
                if message:
                    self.sensor_module.print_screen("rx: {}".format(message), 1, 15)
                    self.process_raspberry_message(message)
                
                # services
                if self.check_mode_active:
                    check_service = self.check_service()
                    self.sensor_module.print_screen("check: {}".format("TRUE" if check_service else "FALSE"), 1, 35)
                    if check_service:
                        data = {'state': 'approved'}
                        self.communication.send_message('check_service', data)
                        self.check_mode_active = False
                    else:
                        data = {'error': 'Sensors or motors not installed'}
                        self.communication.send_message('check_service', data)
                        self.check_mode_active = False
                        
                    
                if self.safety_mode_active:
                    safety_state = self.safety_state_service()
                    self.sensor_module.print_screen("safety: {}".format("TRUE" if safety_state else "FALSE"), 1, 55)
                    
                    if safety_state:
                        data = {
                            'state': 'approved'
                        }
                        self.communication.send_message('safety_service', data)
                        self.safety_mode_active = False
                    else:
                        self.safety_mode_active = False
                    
                if self.scan_mode_active:
                    scan_complete = self.scan_service()
                    self.sensor_module.print_screen("scan: {}".format("TRUE" if self.scan_complete else "error" if self.scan_error else "SCANNING"), 1, 75)
                    if scan_complete:
                        objects_map = self.mapping_module.get_objects_map()
                        data = {
                            'state': 'complete',
                            'objects': objects_map,
                        }
                        self.communication.send_message('scan_service', data)
                        self.scan_mode_active = False
                    else:
                        self.scan_mode_active = False
                        
                    
            except Exception as e:
                self.sensor_module.print_screen("Error: {}".format(str(e)[:20]), 1, 95)
                self.control_module.general_stop()
            
        
            
if __name__ == "__main__":
    arm_process = RoboticServices()
    arm_process.run()
        