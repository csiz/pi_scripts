from time import monotonic
from functools import wraps

def throttle(func, wait):
  """Invoke a function at most once every `wait` seconds.

  Somewhat like `lodash.throttle`, but drops any non-executed calls.
  """
  last_time = monotonic() - wait

  @wraps(func)
  def wrapper(*args, **kwargs):
    nonlocal last_time
    time = monotonic()
    if time >= last_time + wait:
      func(*args, **kwargs)
      last_time = time

  return wrapper

def throttled(wait):
  """Decorator version of `throttle`."""
  return lambda func: throttle(func, wait)