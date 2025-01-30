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
#                       configuration                                            #
# ============================================================================ #
class RobotConfig:
    def __init__(self):
        self.SENSOR_PARAMS = {
            'distance_threshold': 300,
            'min_distance': 50,
            'calibration_time': 2
        }
        
        self.MOTOR_PARAMS = {
            'base': {'speed': 20, 'safety_speed': 10},
            'shoulder': {'speed': 60, 'safety_speed': 10},
            'elbow': {'speed': 20, 'safety_speed': 20},
            'gripper': {'speed': 10, 'safety_speed': 20}
        }
        
        self.SAFETY_PARAMS = {
            'error_threshold': 3,
            'current_limit': 0.3,
            'timeout': 30
        }
        

# ============================================================================ #
#                       state management
# ============================================================================ #
class RobotState:
    INIT = "init"
    ERROR = "error"
    SAFETY_SHOULDER = "safety_shoulder"
    SAFETY_GRIPPER = "safety_gripper"
    SCAN_INIT = "scan_init"
    SCANNING = "scanning"
    SCAN_COMPLETE = "scan_complete"
    IDLE = "idle"
    
    
class LEDColors:
    ERROR = (255, 0, 0)             # Red: Error/Stop
    WARNING = (255, 150, 0)         # Orange: Warning
    READY = (0, 255, 0)             # Green: Ready
    RUNNING = (0, 0, 255)           # Blue: Process Running
    STANDBY = (255, 255, 0)         # Yellow: Standby
    SETUP = (255, 255, 255)         # White: Setup
    OBJECT_DETECTED = (0, 255, 155) # Cyan: Object Detected
    SERIAL = (150, 255, 0)
    
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
        
    def send_message(self, message_type: str, data: dict) -> bool:
        message = {
            'type': message_type,
            'data': data,
            'timestamp': time.time()
        }
        encoded_message = json.dumps(message).encode() + self.message_end
        self.serial_port.write(encoded_message)
        return True
    
    def send_scan_data(self, scan_data: dict):
        return self.send_message('scan_data', scan_data)
        
    def send_status(self, status: str, details: dict = {}):
        data = {'status': status}
        if details:
            data.update(details)
        return self.send_message('status', data)
    
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
    
    def get_orientation(self):
        return self.inertial.orientation(OrientationType.YAW)
    
    # distance methods
    def get_base_distance(self):
        return self.base_distance.object_distance(MM)
    
    def get_base_object_size(self):
        return self.base_distance.object_rawsize()
    
    def get_gripper_distance(self):
        return self.gripper_distance.object_distance(MM)
    
    def get_gripper_object_size(self):
        return self.gripper_distance.object_rawsize()
    
    # bumper methods
    def is_bumper_pressed(self):
        return self.bumper.pressing()
    
    # touchled methods
    def get_value(self):
        return self.touchled.pressing()
    
    def set_color(self, color):
        r, g, b = color
        self.touchled.set_color(r,g,b)
        
    def toggle(self):
        self.touchled.toggle()
        
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
            'is_tracking': False
        }
    
    def process_object_detection(self, angle, object_size):
        if object_size > 0:
            # new object detected
            if not self.current_object['is_tracking']:
                self.current_object['start_angle'] = angle
                self.current_object['is_tracking'] = True
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
            'max_size': self.current_object['max_size']
        })
        
        # reset tracking
        self.current_object['is_tracking'] = False
        self.current_object['max_size'] = 0
        
    def get_objects_map(self):
        return self.objects_map
    
    def process_camera_detection(self, angle, distance, size):
        # Implement camera detection processing logic here
        pass
    

# ============================================================================ #
#                       control module
# ============================================================================ #
class ControlModule:
    def __init__(self, config: RobotConfig):
        self.config = config
        self.base_motor = Motor(Ports.PORT1, True)
        self.shoulder_motor = Motor(Ports.PORT2, True)
        self.elbow_motor = Motor(Ports.PORT3, True)
        self.gripper_motor = Motor(Ports.PORT4, True)
        
        self.speeds = self.config.MOTOR_PARAMS
        
    # base motor methods
    def rotate_base_forward(self):
        self.base_motor.spin(FORWARD, self.speeds['base']['speed'], RPM)
        
    def rotate_base_reverse(self):
        self.base_motor.spin(REVERSE, self.speeds['base']['speed'], RPM)
        
    # shoulder motor methods
    def move_shoulder_forward(self, speed):
        self.shoulder_motor.spin(FORWARD, speed, RPM)
        
    def move_shoulder_reverse(self, speed):
        self.shoulder_motor.spin(REVERSE, speed, RPM)
        
    # elbow motor methods
    def move_elbow_forward(self, speed):
        self.elbow_motor.spin(FORWARD, speed, RPM)
        
    def move_elbow_reverse(self, speed):
        self.elbow_motor.spin(REVERSE, speed, RPM)
        
    # gripper motor methods
    def open_gripper(self, speed):
        self.gripper_motor.spin(FORWARD, speed, RPM)
        
    def close_gripper(self, speed):
        self.gripper_motor.spin(REVERSE, speed, RPM)
        
    def get_gripper_current(self):
        return self.gripper_motor.current()
    
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
    def __init__(self,config:RobotConfig, sensor_module: SensorModule, control_module: ControlModule):
        self.config = config
        self.sensor_module = sensor_module
        self.control_module = control_module
        self.error_count = 0
        self.error_threshold = 3
        
        self.speeds = self.config.MOTOR_PARAMS
        self.safety_params = self.config.SAFETY_PARAMS
    
    # check methods
    def check_motors(self):
        return self.control_module.check_motors()
    
    def check_sensors(self):
        return self.sensor_module.check_sensors()
        
    def check_shoulder_safety(self) -> bool:
        if self.sensor_module.is_bumper_pressed():
            self.control_module.general_stop()
            self.sensor_module.set_color(LEDColors.ERROR)
            self.control_module.move_shoulder_reverse(self.speeds['shoulder']['safety_speed'])
            time.sleep(2)
            self.control_module.general_stop()
            return True
        else:
            self.control_module.move_shoulder_forward(self.speeds['shoulder']['speed'])
            self.sensor_module.set_color(LEDColors.WARNING)
            return False
        
    def check_gripper_safety(self) -> bool:
        gripper_current = self.control_module.get_gripper_current()
        self.control_module.open_gripper(self.speeds['gripper']['safety_speed'])

        if gripper_current > self.safety_params['current_limit']:
            self.control_module.general_stop()
            return True
        else:
            return False

# ============================================================================ #
#                       main module
# ============================================================================ #
class RoboticServices:
    def __init__(self):
        self.config = RobotConfig()
        
        # initialize modules
        self.sensor_module = SensorModule()
        self.perception_module = PerceptionModule(self.sensor_module)
        self.mapping_module = MappingModule()
        self.control_module = ControlModule(self.config)
        self.safety_module = SafetyModule(self.config, self.sensor_module, self.control_module)
        self.communication = CommunicationManager()
        
        
        # -- Variables / banderas internas --
        # "check_service"
        self.checked_once = False
        
        # "safety_service"
        self.safety_mode_active = False
        self.safety_shoulder = False
        self.safety_gripper = False
        self.safety_service_loop = True
        
        # "scan_service"
        self.scan_mode_active = False
        self.scan_start_time = 0
        self.scan_timeout = self.config.SAFETY_PARAMS['timeout']
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
                    self.finish_scan_service(True, "")
                    
    def start_scan_service(self):
        """Initialize scan service"""
        if not self.safety_mode_active or not self.safety_shoulder or not self.safety_gripper:
            return False
        
        if self.scan_complete:
            return False
        
        self.scan_mode_active = True
        self.scan_complete = False
        self.scan_error = False
        
        self.scan_start_time = time.time()
        self.accumulated_rotation = 0
        self.last_angle = self.sensor_module.get_angle()
        
        self.control_module.rotate_base_forward()
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
        self.mapping_module.process_object_detection(current_angle, sensor_data['size'])
        
        # Check if scan is complete
        if self.accumulated_rotation >= 360 or time.time() - self.scan_start_time > self.scan_timeout:
            return True
        
        return False
            
    def finish_scan_service(self, success: bool, error_msg: str = ""):
        """Finish scan service and report results"""
        self.control_module.general_stop()
        
        if success:
            self.scan_complete = True
            self.sensor_module.set_color(LEDColors.READY)
            # Send final map data
            objects_map = self.mapping_module.get_objects_map()
            
    def reset_scan_sequence(self):
        """
        Reset the scan sequence variables.
        Call every time you want to restart it..
        """
        self.scan_mode_active = False
        self.scan_start_time = 0
        self.scan_timeout = self.config.SAFETY_PARAMS['timeout']
        self.accumulated_rotation = 0
        self.last_angle = 0
        self.scan_start = True
        self.scan_update = False
        self.scan_complete = False
        self.scan_error = False
            
    # communication methods
    def process_raspberry_message(self, message:dict):
        """Process JSON messages from Raspberry Pi"""
        if not message or 'type' not in message:
            return
            
        msg_type = message['type']
        msg_data = message.get('data', {})
        
        if msg_type == 'command':
            cmd = msg_data.get('command', '').lower()
            
            if cmd == 'check':
                self.checked_once = True
                
            elif cmd == 'safety':
                self.reset_safety_sequence()
                self.safety_mode_active = True
                
            elif cmd == 'scan':
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
                if self.checked_once:
                    check_service = self.check_service()
                    self.sensor_module.print_screen("check: {}".format("TRUE" if check_service else "FALSE"), 1, 35)
                    
                if self.safety_mode_active:
                    safety_state = self.safety_state_service()
                    self.sensor_module.print_screen("safety: {}".format("TRUE" if safety_state else "FALSE"), 1, 55)
                    
                if self.scan_mode_active:
                    self.scan_service()
                    self.sensor_module.print_screen("scan: {}".format("complete" if self.scan_complete else "error" if self.scan_error else "running"), 1, 75)
                    
            except Exception as e:
                self.sensor_module.print_screen("Error: {}".format(str(e)[:20]), 1, 95)
                self.control_module.general_stop()
            
        
            
if __name__ == "__main__":
    arm_process = RoboticServices()
    arm_process.run()
        