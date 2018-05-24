from time import perf_counter, sleep

from shifty import PWMShiftRegister, ShiftRegister

reg = PWMShiftRegister(SER=17, RCLK=27, SRCLK=22, frequency=200)

while True:
  t = perf_counter()
  vs = [10**(3*sin((t/1.0 + i/8) * 2 * 3.14)-3) for i in range(8)]
  reg.write(vs)
  sleep(0.05)
