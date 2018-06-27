from time import perf_counter, sleep
from math import sin, pi

from gyro import Gyro

if __name__ == "__main__":

  gyro = Gyro()

  gyro.observe("start", lambda time: print("Started at: ", time))

  class Last:
    def __init__(self):
      self.last = None
      self.count = 0

    def __call__(self, item):
      self.last = item
      self.count += 1

    @property
    def count_since_last(self):
      tmp_count = self.count
      self.count = 0
      return tmp_count

  measure = Last()

  gyro.observe("measure", measure)

  gyro.observe("exception", lambda exception: print("Exception:", exception))

  gyro.observe("discarded", lambda: print("Items discarded from FIFO!"))

  try:
    while True:
      gyro.tick()

      if measure.last:
        print("ax: {:6.2f}; nr since last read: {}".format(measure.last.acceleration[0], measure.count_since_last))
      else:
        print("No measures!")

      sleep(0.1)
  except KeyboardInterrupt:
    pass
