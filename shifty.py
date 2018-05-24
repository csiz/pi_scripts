from gpiozero import DigitalOutputDevice, Device, OutputDeviceError
from time import sleep, perf_counter
from threading import Thread


def pulse(pin):
  pin.on()
  pin.off()

class ShiftRegister(Device):
  def __init__(self, SER, SRCLK, RCLK, outputs=8, SRCLR=None, OE=None, pin_factory=None):
    super().__init__(pin_factory=pin_factory)

    self.SER = DigitalOutputDevice(SER, pin_factory=pin_factory)
    self.SRCLK = DigitalOutputDevice(SRCLK, pin_factory=pin_factory)
    self.RCLK = DigitalOutputDevice(RCLK, pin_factory=pin_factory)

    self.SRCLR = DigitalOutputDevice(SRCLR, active_high=False, pin_factory=pin_factory) if SRCLR else None
    self.OE = DigitalOutputDevice(OE, active_high=False, pin_factory=pin_factory) if OE else None

    self._closed = False

    if not isinstance(outputs, int):
      raise TypeError("Number of outputs must be an int.")
    self.outputs = outputs

    self._state = [False] * outputs

    self.clear()


  def close(self):
    if self._closed:
      return

    self.clear()

    self.SER.close()
    self.SRCLK.close()
    self.RCLK.close()

    self.SRCLR is None or self.SRCLR.close()
    self.OE is None or self.OE.close()

    self._closed = True

  def clear(self):
    # If we connected the clearing pin we can just pulse it.
    if self.SRCLR is not None:
      pulse(self.SRCLR)
      pulse(self.RCLK)
      self._state = [False] * self.outputs

    # Nope, pipe in false signals for as many outputs as we have.
    else:
      self.shiftin([False]*self.outputs)


  def shiftin(self, bits):
    bits = [bool(b) for b in bits]

    for bit in bits:
      self.SER.value = bit
      pulse(self.SRCLK)
    pulse(self.RCLK)

    # Shift the previous state to the right and plug these bits in.
    self._state = bits[self.outputs-1::-1] + self._state[:-len(bits)]

  def write(self, bits):
    if len(bits) != self.outputs:
      raise OutputDeviceError(
        "Device initialized with {} outputs, but {} passed in.".format(
          self.outputs, len(bits)))

    self.shiftin(list(reversed(bits)))

  @property
  def value(self):
    return self._state

  @value.setter
  def value(self, bits):
    self.write(bits)



class PWMShiftRegister(Device):
  def __init__(
      self,
      SER, SRCLK, RCLK,
      outputs=8,
      frequency=100,
      SRCLR=None, OE=None, pin_factory=None,
    ):
    """Control shift register pins using pulse width modulation.

    :param float frequency:
      Frequency in Hz at which to pulse the pins.
    """

    super().__init__(pin_factory=pin_factory)

    self._closed = False

    self._underlying = ShiftRegister(SER, SRCLK, RCLK, outputs=outputs, SRCLR=SRCLR, OE=OE, pin_factory=pin_factory)

    if not isinstance(frequency, (float, int)):
      raise TypeError("Frequency must be a number.")
    if frequency < 10:
      raise Warning("Frequency is too low.")

    self._pulse_duration = 1 / frequency

    if not isinstance(outputs, int):
      raise TypeError("Number of outputs must be an int.")
    self.outputs = outputs

    self._state = [0.0] * outputs

    self._running = True
    self._pwm_thread = Thread(target=self._pwm_runner)
    self._pwm_thread.start()

  def _pwm_runner(self):
    # Localise some variables.
    outputs = self.outputs
    pulse = self._pulse_duration
    underlying = self._underlying

    last_toggled = [perf_counter()] * outputs
    state = [False] * outputs

    while self._running:
      values = self._state

      now = perf_counter()

      # Turn off pins that have been on for more than fraction of pulse,
      # and turn on pins that have been off for more than (1-fraction) of pulse.
      next_state = [
        min(now-lt, pulse-1e-9) < pulse*v if s else min(now-lt+1e-9, pulse) > pulse*(1.0-v)
        for s, lt, v
        in zip(state, last_toggled, values)]

      underlying.write(next_state)

      # Takes a while to write it, get time again.
      now = perf_counter()

      last_toggled = [lt if s == ns else now for s, ns, lt in zip(state, next_state, last_toggled)]
      state = next_state


  def write(self, values):
    # Make sure values are numbers in [0, 1].
    values = [float(v) for v in values]
    if not all(0.0 <= v <= 1.0 for v in values):
      raise ValueError("Input values must be in the closed interval [0, 1]; outliers found: {}".format(values))

    if len(values) != self.outputs:
      raise OutputDeviceError(
          "Device initialized with {} outputs, but {} passed in.".format(
              self.outputs, len(values)))

    self._state = values

  def clear(self):
    self._state = [0.0] * self.outputs

  def close(self):
    if self._closed:
      return

    self._running = False
    self._pwm_thread.join()
    self._underlying.close()

    self._closed = True

  @property
  def value(self):
    return self._state

  @value.setter
  def value(self, bits):
    self.write(bits)


