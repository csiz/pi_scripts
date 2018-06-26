from time import perf_counter, sleep
from math import sin, pi

from gyro import Gyro

if __name__ == "__main__":

  gyro = Gyro()

  gyro.observe_any(lambda event, args, kwargs: print(event, args))

  while True:
    gyro.tick()
    sleep(0.05)
