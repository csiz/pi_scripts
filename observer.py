"""Implement the observer pattern for inter thread communication."""

from queue import Queue, Empty, Full

class Dispatcher:
  """Event dispatcher."""

  def __init__(self, maxsize=0):
    """Dispatch up to `maxsize` events, events that are not observed in time are dropped."""
    self.queue = Queue(maxsize=maxsize)

  def trigger(self, event, *args, **kwargs):
    """Trigger the event with parameters."""
    try:
      self.queue.put_nowait((event, args, kwargs))

    # If it was full, discard the top of the queue and try again.
    except Full:
      try:
        self.queue.get_nowait()
      except Empty:
        pass
      # If it's still full this time, something's weird.
      self.queue.put_nowait((event, args, kwargs))

  __call__ = trigger


class Observable:
  """Event listener."""

  def __init__(self, dispatcher):
    """Observe events from dispatcher; should be initialized from `observable`."""
    self.__observers = {}
    self.__any_observers = []
    self.__queue = dispatcher.queue


  def tick(self):
    """Repeatedly call to observe pending events."""
    while True:
      try:
        event, args, kwargs = self.__queue.get_nowait()
      except Empty:
        break

      for observer in self.__observers.get(event, []):
        observer(*args, **kwargs)

      for any_observer in self.__any_observers:
        any_observer(event, args, kwargs)

  def on(self, event, callback):
    """Add callback to be triggered on the event."""
    self.__observers.setdefault(event, []).append(callback)


  def on_any(self, callback):
    """Add callback to be triggered on any event. Invoked with 3 parameters: event, args, kwargs."""
    self.__any_observers.append(callback)


def create_observable(maxsize=0):
  """Create `Observable`, `Dispatcher` pair for event routing."""
  dispatcher = Dispatcher(maxsize=maxsize)
  observable = Observable(dispatcher)

  return observable, dispatcher
