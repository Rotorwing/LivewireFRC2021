from wpilib import DigitalInput, DigitalOutput, AnalogTriggerOutput
import threading
from queue import Queue
from time import sleep, time


class Tachometer:
    def __init__(self, port=None, reverse=False, delay=0, Atrig=None):
        self.q = Queue()
        self.thread_args = (self.q, reverse, port, delay, Atrig)
        self.thread = threading.Thread(target=self._main_loop, daemon=True, args=self.thread_args)
        self.has_run = False

    def _main_loop(self, que, reverse, port, delay, Atrig):
        print("< Started Tach>")
        if Atrig:
            DI = AnalogTriggerOutput(Atrig, (0.1, 0.9))  # idk
        else:
            DI = DigitalInput(port)  # sensor
        last_tick = time()  # time since last trigger
        ticks = 0  # number of total triggers
        last = DI.get()  # last state of the sensor.
        count = last  # number of ticks since last RPM measurement
        TPM = 0  # ticks per minute
        running = True  # Is the deamon running
        ticks_direction = 0  # number of ticks taking into acount direction
        direction = 1  # The current direction of motion

        while not que.empty():  # empty the que
                que.get()

        while running:
            val = DI.get()  # get the sensors value
            if val != last and not val == reverse:  #
                ticks += 1
                count += 1
                ticks += direction

            # Read everything from the que to empty it and manage any commands
            while not que.empty():
                data = que.get()
                if data[0] == "FLAG":
                    if data[1] == "KILL":
                        print("KILL")
                        running = False
                elif data[0] == "DIR":
                    direction = data[1]
                elif data[0] == "TICKS":
                    ticks = data[1]
                    ticks_direction = data[1]

            if count >= 2:
                elapsed = time()-last_tick
                TPM = count/elapsed*60
                count = 0
                last_tick = time()

            que.put(("DATA", ticks, TPM, ticks_direction))
            last = val
            sleep(delay)

    def start_thread(self):
        if self.has_run:
            self.thread = threading.Thread(target=self._main_loop, daemon=True, args=self.thread_args)
        self.thread.start()
        self.has_run = True

    def stop_thread(self):
        self.q.put(("FLAG", "KILL"))

    def get_ticks(self):
        data = self.q.get()
        if data[0] == "DATA":  # If this is a data packet, return the tick count
            return data[1]
        else:  # Otherwise, put the packet back and return 0
            self.q.put(data)
            return 0

    def get_TPM(self):
        data = self.q.get()
        if data[0] == "DATA":  # If this is a data packet, return the TPM
            return data[2]
        else:  # Otherwise, put the packet back and return 0
            self.q.put(data)
            return 0

    def set_direction(self, unit):
        if unit != 0:
            direction = unit/abs(unit)
        else:
            direction = 0

        self.q.put(("DIR", direction))

    def get_ticks_caleb(self):
        """Returns the number of ticks since start taking into account direction change."""
        data = self.q.get()
        if data[0] == "DATA":
            return data[3]
        else:
            return None

    def set_ticks(self, value):
        self.q.put(("TICKS", value))

    def zero_ticks(self):
        self.set_ticks(0)

