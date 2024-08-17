import numpy as np
import time
from finger_control import FingerCtrl

class FingerCtrlDisplay(FingerCtrl):
    def __init__(self, uart_ports, baudrate):
        super().__init__(uart_ports, baudrate)
    
    def generate_sine_wave(self, amplitude, frequency, phase, time_step):
        """
        生成正弦波速度
        """
        return amplitude * np.sin(2 * np.pi * frequency * time_step + phase)
    
    def generate_square_wave(self, amplitude, frequency, phase, time_step):
        """
        生成矩形波速度
        """
        return amplitude * np.sign(np.sin(2 * np.pi * frequency * time_step + phase))

    def run_wave_pattern(self, wave_type='sine', amplitude=4, frequency=0.5, duration=10):
        """
        让三个手指的第一个电机做相差三分之一周期的波形速度
        """
        start_time = time.time()
        time_step = 0

        while time.time() - start_time < duration:
            time_step = time.time() - start_time
            if wave_type == 'sine':
                speeds = [
                    [self.generate_sine_wave(amplitude, frequency, 0, time_step), 0],
                    [self.generate_sine_wave(amplitude, frequency, 2 * np.pi / 3, time_step), 0],
                    [self.generate_sine_wave(amplitude, frequency, 4 * np.pi / 3, time_step), 0],
                ]
            elif wave_type == 'square':
                speeds = [
                    [self.generate_square_wave(amplitude, frequency, 0, time_step), 0],
                    [self.generate_square_wave(amplitude, frequency, 2 * np.pi / 3, time_step), 0],
                    [self.generate_square_wave(amplitude, frequency, 4 * np.pi / 3, time_step), 0],
                ]
            else:
                raise ValueError("Unsupported wave type. Use 'sine' or 'square'.")
            
            self.set_all_speeds(speeds)
            for i in range(self.finger_num):
                self.N32[i].send_msg()

            time.sleep(0.05)  # 控制更新频率

        self.close()

    def run_wave_pattern_forever(self, wave_type='sine', amplitude=4, frequency=0.5):
        """
        让三个手指的第一个电机做相差三分之一周期的波形速度
        """
        time_step = 0
        while True:
            time_step = time.time()
            if wave_type == 'sine':
                speeds = [
                    [self.generate_sine_wave(amplitude, frequency, 0, time_step), 0],
                    [self.generate_sine_wave(amplitude, frequency, 2 * np.pi / 3, time_step), 0],
                    [self.generate_sine_wave(amplitude, frequency, 4 * np.pi / 3, time_step), 0],
                ]
            elif wave_type == 'square':
                speeds = [
                    [self.generate_square_wave(amplitude, frequency, 0, time_step), 0],
                    [self.generate_square_wave(amplitude, frequency, 2 * np.pi / 3, time_step), 0],
                    [self.generate_square_wave(amplitude, frequency, 4 * np.pi / 3, time_step), 0],
                ]
            else:
                raise ValueError("Unsupported wave type. Use 'sine' or 'square'.")
            
            self.set_all_speeds(speeds)
            for i in range(self.finger_num):
                self.N32[i].send_msg()

            time.sleep(0.05)


if __name__ == "__main__":
    uart_ports = ["/dev/ttyS2", "/dev/ttyS1", "/dev/ttyS3"]
    baudrate = 230400
    fingers = FingerCtrlDisplay(uart_ports, baudrate)
    
    # 运行正弦波模式，振幅10，频率0.5Hz，持续10秒
    # fingers.run_wave_pattern(wave_type='sine', amplitude=4, frequency=0.5, duration=10)
    
    # 运行矩形波模式，振幅10，频率0.5Hz，持续10秒
    # fingers.run_wave_pattern(wave_type='square', amplitude=10, frequency=0.5, duration=10)

    try :
        fingers.run_wave_pattern_forever(wave_type='square', amplitude=4, frequency=0.5)
    except KeyboardInterrupt:
        fingers.close()
        print("Program exit")
