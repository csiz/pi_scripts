from gpiozero import DigitalOutputDevice, Device
from time import sleep

ns = 10**-9

def pulse(pin, duration):
  pin.on()
  sleep(duration)
  pin.off()

class ShiftRegister(Device):
  def __init__(self, SER, SRCLK, RCLK, SRCLR=None, OE=None, pin_factory=None):
    super().__init__(pin_factory=pin_factory)

    self.SER = DigitalOutputDevice(SER, pin_factory=pin_factory)
    self.SRCLK = DigitalOutputDevice(SRCLK, pin_factory=pin_factory)
    self.RCLK = DigitalOutputDevice(RCLK, pin_factory=pin_factory)

    self.SRCLR = DigitalOutputDevice(SRCLR, active_high=False, pin_factory=pin_factory) if SRCLR else None
    self.OE = DigitalOutputDevice(OE, active_high=False, pin_factory=pin_factory) if OE else None

  def close(self):
    self.SER.close()
    self.SRCLK.close()
    self.RCLK.close()

    self.SRCLR is None or self.SRCLR.close()
    self.OE is None or self.OE.close()

  def clear(self):

    assert self.SRCLR is not None

    pulse(self.SRCLR, 15*ns)
    sleep(15*ns)
    pulse(self.RCLK, 25*ns)

  def shiftin(self, bits):
    for bit in bits:
      assert isinstance(bit, bool)
      self.SER.value = bit
      sleep(30*ns)
      pulse(self.SRCLK, 25*ns)

    pulse(self.RCLK, 25*ns)


# reg = shifty.ShiftRegister(SER=17, RCLK=27, SRCLK=22, SRCLR=23)
