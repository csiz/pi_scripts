from time import perf_counter, sleep
from math import sin, pi

from gyro import Gyro

if __name__ == "__main__":

  gyro = Gyro()

  gyro.observe("start", lambda time: print("Started at: ", time))

  class Last:
    def __init__(self):
      self.last = None
    def __call__(self, item):
      self.last = item

  measure = Last()

  gyro.observe("measure", measure)

  gyro.observe("exception", lambda exception: print("Exception:", exception))

  gyro.observe("discarded", lambda: print("Items discarded from FIFO!"))

  try:
    while True:
      gyro.tick()

      if measure.last:
        print("ax: {:.2f}".format(measure.last.acceleration[0]))
      else:
        print("No measures!")

      sleep(0.2)
  except KeyboardInterrupt:
    pass
