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
#                       base interfaces                                         
# ============================================================================ #
class BaseModule:
    def initialize(self) -> bool:
        """Initialize module and return success status"""
        return True
    
    def check_health(self) -> bool:
        """Check if module is healthy"""
        return True
    

class StateObserver:
    def on_state_change(self, old_state: str, new_state: str):
        """Called when state changes"""
        pass
    
    def on_error(self, error_type: str, message: str):
        """Called when error occurs"""
        pass
    
    
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
    

class StateManager:
    def __init__(self):
        self.current_state = RobotState.INIT
        self.observers = []
        
    def add_observer(self, observer: StateObserver):
        self.observers.append(observer)
        
    def transition_to(self, new_state: str):
        old_state = self.current_state
        self.current_state = new_state
        for observer in self.observers:
            observer.on_state_change(old_state, new_state)
    
    def get_current_state(self) -> str:
        return self.current_state
    
    
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
class CommunicationManager(BaseModule):
    def __init__(self, config: RobotConfig):
        self.config = config
        self.serial_port = None
        self.buffer = bytearray()
        self.message_end = b'\n'
        
    def initialize(self) -> bool:
        try:
            self.serial_port = open('/dev/serial1', 'rb+')
            return True
        except Exception as e:
            return False
        
    def send_message(self, message_type: str, data: dict) -> bool:
        try:
            message = {
                'type': message_type,
                'data': data,
                'timestamp': time.time()
            }
            encoded_message = json.dumps(message).encode() + self.message_end
            self.serial_port.write(encoded_message)
            return True
        except Exception as e:
            return False
        
    def read_message(self):
        byte = self.serial_port.read(1)
        if byte:
            self.buffer.extend(byte)
            if byte == self.message_end:
                message = self.buffer[:-1].decode().strip()
                self.buffer = bytearray()
                return json.loads(message)
                #return self._parse_message(message)
    
    def _parse_message(self, raw_message: str) -> dict:
        """convert string to dict"""
        parts = raw_message.split(',')
        message_dict = {}
        for part in parts:
            if ':' in part:
                key, value = part.split(':', 1)
                message_dict[key.strip()] = self._parse_value(value.strip())
        return message_dict
    
    def _parse_value(self, value: str):
        """convert data type"""
        try:
            return int(value)
        except ValueError:
            try:
                return float(value)
            except ValueError:
                if value.lower() == 'true':
                    return True
                elif value.lower() == 'false':
                    return False
                return value
        
            
    def send_sensor_data(self, sensor_data: dict):
        return self.send_message('sensor_data', sensor_data)
        
    def send_position_data(self, position_data: dict):
        return self.send_message('position_data', position_data)
        
    def close(self):
        if self.serial_port:
            self.serial_port.close()
            

# ============================================================================ #
#                       sensor module
# ============================================================================ #
class SensorModule(BaseModule):
    def __init__(self, config: RobotConfig):
        self.config = config
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
class ControlModule(BaseModule):
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
class SafetyModule(BaseModule, StateObserver):
    def __init__(self,config:RobotConfig, sensor_module: SensorModule, control_module: ControlModule):
        self.config = config
        self.sensor_module = sensor_module
        self.control_module = control_module
        self.error_count = 0
        self.error_threshold = 3
        
        self.speeds = self.config.MOTOR_PARAMS
        self.safety_params = self.config.SAFETY_PARAMS
        
    # states
    def on_state_change(self, old_state: str, new_state: str):
        if new_state == RobotState.ERROR:
            self.handle_error_state()
            
    def handle_error_state(self):
        self.control_module.general_stop()
        self.sensor_module.set_color(LEDColors.ERROR)
    
    # check methods
    def check_motors(self):
        return self.control_module.check_motors()
    
    def check_sensors(self):
        return self.sensor_module.check_sensors()
        
    def check_shoulder_safety(self, current_state: str) -> str:
        if self.sensor_module.is_bumper_pressed():
            self.control_module.general_stop()
            self.sensor_module.set_color(LEDColors.ERROR)
            self.control_module.move_shoulder_reverse(self.speeds['shoulder']['safety_speed'])
            time.sleep(2)
            self.control_module.general_stop()
            return RobotState.SAFETY_GRIPPER
        else:
            self.control_module.move_shoulder_forward(self.speeds['shoulder']['speed'])
            self.sensor_module.set_color(LEDColors.WARNING)
            return current_state
        
    def check_gripper_safety(self, current_state: str) -> str:
        gripper_current = self.control_module.get_gripper_current()
        self.control_module.open_gripper(self.speeds['gripper']['safety_speed'])

        if gripper_current > self.safety_params['current_limit']:
            self.control_module.general_stop()
            return RobotState.SCAN_INIT
        else: 
            return current_state

# ============================================================================ #
#                       main module
# ============================================================================ #
class RoboticArmSystem:
    def __init__(self):
        self.config = RobotConfig()
        self.state_manager = StateManager()
        
        # initialize modules
        self.sensor_module = SensorModule(self.config)
        self.perception_module = PerceptionModule(self.sensor_module)
        self.mapping_module = MappingModule()
        self.control_module = ControlModule(self.config)
        self.safety_module = SafetyModule(self.config, self.sensor_module, self.control_module)
        self.communication = CommunicationManager(self.config)
        
        # register observers
        self.state_manager.add_observer(self.safety_module)
        
        # initialize state
        self.scan_start_time = 0
        self.scan_timeout = self.config.SAFETY_PARAMS['timeout']
        self.accumulated_rotation = 0
        self.last_angle = 0
    
        # calibrate inertial sensor
        #self.sensor_module.calibrate_intertial_sensor()

    # state machine
    def update_state(self):
        current_state = self.state_manager.get_current_state()
        
        if current_state == RobotState.ERROR:
            return
        
        # process incoming messages
        msggggggg = self.communication.read_message()

        
        try:
            # 1. check motors
            if current_state == RobotState.INIT:
                if self.safety_module.check_sensors() and self.safety_module.check_motors():
                    self.sensor_module.set_color(LEDColors.READY)
                    time.sleep(1)
                    self.state_manager.transition_to(RobotState.SAFETY_SHOULDER)   
                    
            # 2. safety check
            elif current_state == RobotState.SAFETY_SHOULDER:
                new_state = self.safety_module.check_shoulder_safety(current_state=current_state)
                if new_state != current_state:
                    self.state_manager.transition_to(new_state)
                
            elif current_state == RobotState.SAFETY_GRIPPER:
                new_state = self.safety_module.check_gripper_safety(current_state=current_state)
                if new_state != current_state:
                    self.state_manager.transition_to(new_state)
                
            # 3. scan
            elif current_state == RobotState.SCAN_INIT:
                self.start_scan()
                self.state_manager.transition_to(RobotState.SCANNING)
                
            elif current_state == RobotState.SCANNING:
                self.update_scan()
                
            elif current_state == RobotState.SCAN_COMPLETE:
                objects = self.mapping_module.get_objects_map()
                total_objects = len(objects)
                self.sensor_module.set_color(LEDColors.READY)
                self.sensor_module.print_screen("Objects: {}".format(total_objects), 1, 15)
                
        except Exception as e:
            self.state_manager.transition_to(RobotState.ERROR)
    
    # scan methods
    def start_scan(self):
        self.scan_start_time = time.time()
        self.accumulated_rotation = 0
        self.last_angle = self.sensor_module.get_angle()
        self.control_module.rotate_base_forward()
        self.sensor_module.set_color(LEDColors.RUNNING)
        
    def update_scan(self):
        current_angle = self.sensor_module.get_angle()
        
        # calculate rotation
        delta = current_angle - self.last_angle
        if delta > 180: delta -= 360
        elif delta < -180: delta += 360
        
        self.accumulated_rotation += delta
        self.last_angle = current_angle
        
        # process sensor data
        sensor_data = self.perception_module.process_sensor_data()
        self.mapping_module.process_object_detection(current_angle, sensor_data['size'])
        
        # check if scan is complete
        if abs(self.accumulated_rotation) >= 360 or time.time() - self.scan_start_time > self.scan_timeout:
            self.control_module.general_stop()
            self.state_manager.transition_to(RobotState.SCAN_COMPLETE)
            
    # communication methods
    def process_raspberry_message(self, message: dict):
        """Process messages received from Raspberry Pi"""
        if message['type'] == 'object_detected':
            # Update mapping with object detection from camera
            self.mapping_module.process_camera_detection(
                message['data']['angle'],
                message['data']['distance'],
                message['data']['size']
            )
            
        elif message['type'] == 'command':
            # Process control commands
            if message['data']['command'] == 'stop':
                self.control_module.general_stop()
            
    # run method
    def run(self):
        if self.communication.initialize():
            while True:
                try:
                    self.update_state()
                    time.sleep(0.02)
                except Exception as e:
                    self.sensor_module.set_color(LEDColors.ERROR)
                    self.control_module.general_stop()
                    self.state = RobotState.ERROR
                    break
        self.communication.close()
        
            
if __name__ == "__main__":
    arm_process = RoboticArmSystem()
    arm_process.run()
        