import time
from receiver import Receiver

loop_counter = 0

receiver = Receiver()
receiver.start()

while True:
    loop_counter += 1

    if receiver.is_ready():
        print(receiver.get_data())
    
    print(loop_counter)

    time.sleep(0.1)