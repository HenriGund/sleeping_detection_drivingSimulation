import time
import threading
import serial
import pyvjoy
import pygame

SERIAL_PORT = "COM4"
SERIAL_BAUD = 115200

JOYSTICK_AXIS_INDEX = 2
POLL_INTERVAL = 0.02

VJOY_AXIS = pyvjoy.HID_USAGE_RX

REDUCE_RATE_PER_SEC = 0.6
RECOVER_RATE_PER_SEC = 1.5
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0

PV_MIN = 0x0000
PV_MAX = 0x8000

CMD_BUZZ_ON = "BUZZ_ON\n"
CMD_BUZZ_OFF = "BUZZ_OFF\n"

is_asleep = False
running = True

def scale_value(v: float) -> int:
    v = max(0.0, min(1.0, v))
    return int(PV_MIN + (PV_MAX - PV_MIN) * v)


class BuzzerController:
    def __init__(self, port, baud=115200, timeout=1.0):
        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            time.sleep(2.0)
            self.lock = threading.Lock()
        except:
            self.ser = None
            self.lock = threading.Lock()
        self.active = False

    def buzz_on(self):
        if self.ser and not self.active:
            with self.lock:
                self.ser.write(CMD_BUZZ_ON.encode())
                self.active = True

    def buzz_off(self):
        if self.ser and self.active:
            with self.lock:
                self.ser.write(CMD_BUZZ_OFF.encode())
                self.active = False

    def close(self):
        if self.ser:
            self.ser.close()


class PhysicalJoystick:
    def __init__(self, axis_index):
        pygame.init()
        pygame.joystick.init()
        self.axis_index = axis_index
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def read_throttle(self) -> float:
        if not self.joystick:
            return 0.0
        pygame.event.pump()
        raw = self.joystick.get_axis(self.axis_index)
        return max(0.0, min(1.0, (raw + 1.0) / 2.0))


class VirtualJoystick:
    def __init__(self, device_id=1):
        try:
            self.j = pyvjoy.VJoyDevice(device_id)
        except:
            self.j = None

    def set_throttle(self, normalized: float):
        if self.j:
            self.j.set_axis(VJOY_AXIS, scale_value(normalized))


def controller(buzzer, phys, virt):
    global is_asleep, running
    current_throttle = 0.0
    last_time = time.time()

    while running:
        dt = time.time() - last_time
        last_time = time.time()

        real_throttle = phys.read_throttle()

        if is_asleep:
            buzzer.buzz_on()
            target = MIN_THROTTLE
            current_throttle = max(target, current_throttle - REDUCE_RATE_PER_SEC * dt)
        else:
            buzzer.buzz_off()
            target = real_throttle
            if current_throttle < target:
                current_throttle = min(target, current_throttle + RECOVER_RATE_PER_SEC * dt)
            else:
                current_throttle = max(target, current_throttle - RECOVER_RATE_PER_SEC * dt)

        virt.set_throttle(current_throttle)
        time.sleep(POLL_INTERVAL)


def demo_asleep_toggle():
    global is_asleep, running
    while running:
        time.sleep(10)
        is_asleep = not is_asleep
        print("is_asleep:", is_asleep)


def main():
    global running

    buzzer = BuzzerController(SERIAL_PORT, SERIAL_BAUD)
    phys = PhysicalJoystick(JOYSTICK_AXIS_INDEX)
    virt = VirtualJoystick()

    t1 = threading.Thread(target=controller, args=(buzzer, phys, virt), daemon=True)
    t1.start()

    t2 = threading.Thread(target=demo_asleep_toggle, daemon=True)
    t2.start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        running = False
        buzzer.buzz_off()
        buzzer.close()
        pygame.quit()


if __name__ == "__main__":
    main()
