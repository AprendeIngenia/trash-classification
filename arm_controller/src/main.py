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

# ============================================================================ #
#                       constants module
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
        return (self.inertial.installed() and
                self.gripper_distance.installed() and
                self.base_distance.installed() and
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
    

# ============================================================================ #
#                       control module
# ============================================================================ #
class ControlModule:
    def __init__(self):
        self.base_motor = Motor(Ports.PORT1, True)
        self.shoulder_motor = Motor(Ports.PORT2, True)
        self.elbow_motor = Motor(Ports.PORT3, True)
        self.gripper_motor = Motor(Ports.PORT4, True)
        
        self.speeds = {
            'base': 20,
            'shoulder':30,
            'elbow': 20,
            'gripper': 10
        }
        
    # base motor methods
    def rotate_base_forward(self):
        self.base_motor.spin(FORWARD, self.speeds['base'], RPM)
        
    def rotate_base_reverse(self):
        self.base_motor.spin(REVERSE, self.speeds['base'], RPM)
        
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
    def __init__(self, sensor_module: SensorModule, control_module: ControlModule):
        self.sensor_module = sensor_module
        self.control_module = control_module
        self.error_count = 0
        self.error_threshold = 3
        
        self.safety_forward_speeds = {
            'base': 10,
            'shoulder':60,
            'elbow': 20,
            'gripper': 20
        }
        self.safety_reverse_speeds = {
            'base': 10,
            'shoulder': 10,
            'elbow': 20,
            'gripper': 10
        }
    
    # check methods
    def check_motors(self):
        return self.control_module.check_motors()
    
    def check_sensors(self):
        return self.sensor_module.check_sensors()
        
    def check_shoulder_safety(self, current_state: str) -> str:
        if self.sensor_module.is_bumper_pressed():
            self.control_module.general_stop()
            self.sensor_module.set_color(LEDColors.ERROR)
            self.control_module.move_shoulder_reverse(self.safety_reverse_speeds['shoulder'])
            time.sleep(2)
            self.control_module.general_stop()
            return RobotState.SAFETY_GRIPPER
        else:
            self.control_module.move_shoulder_forward(self.safety_forward_speeds['shoulder'])
            self.sensor_module.set_color(LEDColors.WARNING)
            return current_state
        
    def check_gripper_safety(self, current_state: str) -> str:
        gripper_current = self.control_module.get_gripper_current()
        self.control_module.open_gripper(self.safety_forward_speeds['gripper'])

        if gripper_current > 0.3:
            self.control_module.general_stop()
            return RobotState.SCAN_INIT
        else: 
            return current_state

# ============================================================================ #
#                       main module
# ============================================================================ #
class RoboticArmSystem:
    def __init__(self):
        self.sensor_module = SensorModule()
        self.perception_module = PerceptionModule(self.sensor_module)
        self.mapping_module = MappingModule()
        self.control_module = ControlModule()
        self.safety_module = SafetyModule(self.sensor_module, self.control_module)
        
        self.state = RobotState.INIT
        self.scan_start_time = 0
        self.scan_timeout = 30
        self.accumulated_rotation = 0
        self.last_angle = 0
        
        # calibrate inertial sensor
        #self.sensor_module.calibrate_intertial_sensor()

        
    def update_state(self):
        if self.state == RobotState.ERROR:
            self.control_module.general_stop()
            self.sensor_module.set_color(LEDColors.ERROR)
            return
        
        # 1. check motors
        if self.state == RobotState.INIT:
            if self.safety_module.check_sensors() and self.safety_module.check_motors():
                self.sensor_module.set_color(LEDColors.READY)
                time.sleep(1)
                self.state = RobotState.SAFETY_SHOULDER
                
        # 2. safety check
        elif self.state == RobotState.SAFETY_SHOULDER:
            self.state = self.safety_module.check_shoulder_safety(self.state)
            self.sensor_module.set_color(LEDColors.WARNING)
            
        elif self.state == RobotState.SAFETY_GRIPPER:
            self.state = self.safety_module.check_gripper_safety(self.state)
            self.sensor_module.set_color(LEDColors.WARNING)
            
        # 3. scan
        elif self.state == RobotState.SCAN_INIT:
            self.start_scan()
            self.state = RobotState.SCANNING
            
        elif self.state == RobotState.SCANNING:
            self.update_scan()
            
        elif self.state == RobotState.SCAN_COMPLETE:
            objects = self.mapping_module.get_objects_map()
            total_objects = len(objects)
            self.sensor_module.set_color(LEDColors.READY)
            self.sensor_module.print_screen("Objects: {}".format(total_objects), 1, 15)
                
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
            self.state = RobotState.SCAN_COMPLETE
            
    def run(self):
        while True:
            try:
                self.update_state()
                time.sleep(0.02)
            except Exception as e:
                self.sensor_module.set_color(LEDColors.ERROR)
                self.control_module.general_stop()
                self.state = RobotState.ERROR
                break
            
            
if __name__ == "__main__":
    arm_process = RoboticArmSystem()
    arm_process.run()
        