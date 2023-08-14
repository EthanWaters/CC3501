import spidev
import time
from threading import Thread

from receiver import Receiver


class Backmotor:
    current_val = 128
    increment_val = 1
    is_at_target = True

    def __init__(self, spi_bus, spi_device, spi, name):
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.spi = spi
        self.name = name

    def write_val(self, val):
        self.is_at_target = False
        inc = -self.increment_val if val < self.current_val else self.increment_val
        thread = Thread(target=async_write_val, args=(self.current_val, val, inc, self))
        thread.start()
        self.current_val = val


def async_write_val(current_val, val, inc, backmotor):
    spacer = int(inc/abs(inc))

    polarity = -1 if val < current_val else 1
    chunk_size = 5
    chunk = int(abs(val - current_val) / chunk_size) * polarity

    for i in range(current_val, val + spacer, chunk):
        # The spacer adds or subtracts 1 to ensure it is included by the range.
        backmotor.spi.open(backmotor.spi_bus, backmotor.spi_device)
        backmotor.spi.max_speed_hz = 1000000
        backmotor.spi.xfer([i])

        print(f"{backmotor.name}: {i}")

        backmotor.current_val = i
        time.sleep(0.05)

    remainder = (val % chunk) + int(val / chunk) * chunk

    if remainder != 0:
        backmotor.spi.xfer([remainder])
        print(f"{backmotor.name}: {remainder}")

    backmotor.spi.close()
    backmotor.is_at_target = True


def async_log(motors=[], file_name="log.txt", rate=0.1):
    global stop_logging

    # receiver = Receiver()
    # receiver.start()

    with open(file_name, "a") as log_file:
        while not stop_logging:
            for motor in motors:
                log_file.write(f"{time.strftime('%Y%m%d-%H%M%S')}, {motor.name}, {motor.current_val}\n")

            # if receiver.is_ready():
            #     data = receiver.get_data()
            #     print(data)
            #     log_file.write(f"time.strftime('%Y%m%d-%H%M%S'), localisation, {data}\n")

            log_file.flush()
            
            time.sleep(rate)


def wait_for_output(motors):
    while not all([motor.is_at_target for motor in motors]):
        pass


if __name__ == "__main__":
    stop_logging = False

    # Define SPI ports:
    spi1 = spidev.SpiDev()
    spi2 = spidev.SpiDev()

    # Define motors:
    motor1 = Backmotor(0, 0, spi1, "L-M")
    motor2 = Backmotor(0, 1, spi2, "R-M")

    motors = [motor1, motor2]

    # Start logging:
    filename = f"boat_log_{time.strftime('%d-%b-%Y (%H-%M-%S)')}.txt"
    log_thread = Thread(target=async_log, args=(motors, filename))
    log_thread.start()

    # Do some stuff with the motors:
    motor1.write_val(140)
    motor2.write_val(120)

    wait_for_output(motors)
    time.sleep(2)

    motor1.write_val(0)
    motor2.write_val(0)

    # time.sleep(5)
    wait_for_output(motors)
    time.sleep(0.5)

    stop_logging = True
