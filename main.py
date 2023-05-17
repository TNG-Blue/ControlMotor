from machine import Pin, PWM
import time

class DCMotorController:
    def __init__(self, pwm_pin_1, pwm_pin_2, output_pin_1, output_pin_2):
        self.pwm_1 = PWM(Pin(pwm_pin_1, Pin.OUT))
        self.pwm_2 = PWM(Pin(pwm_pin_2, Pin.OUT))
        self.output_1 = Pin(output_pin_1, Pin.OUT)
        self.output_2 = Pin(output_pin_2, Pin.OUT)
        self.dc_duty_cycle = 0
        self.output_1_state = True
        self.output_2_state = False
        self.pwm_1_state = False
        self.pwm_2_state = True
        self.button_4_pressed = False

        self.pwm_1.freq(10)
        self.pwm_2.freq(10)
        self.set_dc_duty_cycle(self.dc_duty_cycle)
        self.set_output_pins(self.output_1_state, self.output_2_state)
        self.set_pwm_pins(self.pwm_1_state, self.pwm_2_state)

    def set_dc_duty_cycle(self, duty_cycle):
        self.dc_duty_cycle = duty_cycle
        self.pwm_1.duty(int(duty_cycle * 1023 / 100))
        self.pwm_2.duty(int(duty_cycle * 1023 / 100))

    def set_output_pins(self, pin_1_state, pin_2_state):
        self.output_1_state = pin_1_state
        self.output_2_state = pin_2_state
        self.output_1.value(pin_1_state)
        self.output_2.value(pin_2_state)

    def set_pwm_pins(self, pwm_1_state, pwm_2_state):
        self.pwm_1_state = pwm_1_state
        self.pwm_2_state = pwm_2_state
        if pwm_1_state:
            self.pwm_1.duty(int(self.dc_duty_cycle * 1023 / 100))
        else:
            self.pwm_1.duty(0)
        if pwm_2_state:
            self.pwm_2.duty(int(self.dc_duty_cycle * 1023 / 100))
        else:
            self.pwm_2.duty(0)

    def handle_button_press(self, button_num):
        if button_num == 4:
            self.button_4_pressed = not self.button_4_pressed
            if self.button_4_pressed:
                self.set_output_pins(False, True)
                if self.dc_duty_cycle > 0: # nếu động cơ hiện đang chạy về phía trước
                    self.set_pwm_pins(True, False)
                else:
                    self.set_pwm_pins(False, True)
            else:
                self.set_output_pins(True, False)
                if self.dc_duty_cycle > 0: # nếu động cơ hiện đang chạy về phía trước
                    self.set_pwm_pins(False, True)
                else:
                    self.set_pwm_pins(True, False) # đặt động cơ để chạy về phía trước
        else:
            if not self.button_4_pressed: # nếu cầu H không ở chế độ lùi
                if button_num == 1:
                    self.set_dc_duty_cycle(25)
                    self.set_pwm_pins(False, True)
                elif button_num == 2:
                    self.set_dc_duty_cycle(50)
                    self.set_pwm_pins(False, True)
                elif button_num == 3:
                    self.set_dc_duty_cycle(75)
                    self.set_pwm_pins(False, True)
            else: 
                if button_num == 1:
                    self.set_dc_duty_cycle(25)
                    self.set_pwm_pins(True, False)
                elif button_num == 2:
                    self.set_dc_duty_cycle(50)
                    self.set_pwm_pins(True, False)
                elif button_num == 3:
                    self.set_dc_duty_cycle(75)
                self.set_pwm_pins(True, False)
                
class StepperMotorController:
    def __init__(self, in1_pin, in2_pin, in3_pin, in4_pin):
        self.IN1 = Pin(in1_pin, Pin.OUT)
        self.IN2 = Pin(in2_pin, Pin.OUT)
        self.IN3 = Pin(in3_pin, Pin.OUT)
        self.IN4 = Pin(in4_pin, Pin.OUT)

        self.wave_step_seq = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
        self.half_step_seq = [[1,0,0,0], [1,1,0,0], [0,1,0,0], [0,1,1,0], [0,0,1,0], [0,0,1,1], [0,0,0,1], [1,0,0,1]]
        self.full_step_seq = [[1,1,0,0], [0,1,1,0], [0,0,1,1], [1,0,0,1]]

    def _set_sequence(self, mode):
        if mode == 0:
            return self.wave_step_seq
        elif mode == 1:
            return self.half_step_seq
        elif mode == 2:
            return self.full_step_seq
        else:
            return self.wave_step_seq
        
    def step(self, mode, direction):
        seq = self._set_sequence(mode)
        if direction == 1:
            for i in range(len(seq)):
                self.IN1.value(seq[i][0])
                self.IN2.value(seq[i][1])
                self.IN3.value(seq[i][2])
                self.IN4.value(seq[i][3])
                time.sleep(0.005)
        elif direction == -1:
            for i in range(len(seq)-1, -1, -1):
                self.IN1.value(seq[i][0])
                self.IN2.value(seq[i][1])
                self.IN3.value(seq[i][2])
                self.IN4.value(seq[i][3])
                time.sleep(0.005)

class ServoMotorController:
    def __init__(self, pin):
        # Khởi tạo các thuộc tính
        self.servo_pin = Pin(pin)
        self.pwm = PWM(self.servo_pin)
        self.pwm.freq(50)
        self.min_duty = 20
        self.max_duty = 120
        self.current_duty = 0
    
    def move_to(self, angle):
        # Tính toán duty cycle từ góc của động cơ servo
        duty = self.min_duty + angle / 180 * (self.max_duty - self.min_duty)
        # Thiết lập duty cycle cho tín hiệu PWM
        self.pwm.duty(int(duty))
        # Đợi một chút để động cơ servo có thể di chuyển đến góc mới
        time.sleep_ms(500)

    def move_left(self):
        self.move_to(45)
        
    def move_center(self):
        self.move_to(85)
    
    def move_right(self):
        self.move_to(115)
    
    def move_wave(self):
        self.move_to(10)

motor_controller = DCMotorController(8, 9, 10, 11)
motor = StepperMotorController(12, 13, 14, 15)
servo = ServoMotorController(16)

Pin1 = Pin(4, Pin.IN, Pin.PULL_UP)
Pin2 = Pin(5, Pin.IN, Pin.PULL_UP)
Pin3 = Pin(6, Pin.IN, Pin.PULL_UP)
Pin4 = Pin(7, Pin.IN, Pin.PULL_UP)

switch1 = Pin(1, Pin.IN)
switch2 = Pin(2, Pin.IN)
switch3 = Pin(3, Pin.IN)

mode = 0
direction = 1

while True:
    if switch1.value() == 1:
        if not Pin1.value():
            motor_controller.handle_button_press(1)
        elif not Pin2.value():
            motor_controller.handle_button_press(2)
        elif not Pin3.value():
            motor_controller.handle_button_press(3)
        elif not Pin4.value():
            motor_controller.handle_button_press(4)

    if switch2.value() == 1:
        if Pin1.value() == 0:
            mode = 0
        elif Pin2.value() == 0:
            mode = 1
        elif Pin3.value() == 0:
            mode = 2
        elif Pin4.value() == 0:
            direction = -direction
        motor.step(mode, direction)
        
    if switch3.value() == 1:
        if not Pin1.value():
            servo.move_left()
        elif not Pin2.value():
            servo.move_center()
        elif not Pin3.value():
            servo.move_right()
        elif not Pin4.value():
            servo.move_wave()
