"""Implement the observer pattern between threads."""

from queue import Queue, Empty

class EventRouter:
  def __init__(self):
    """Event router between threads."""

    self.observers = {}
    self.any_observers = []
    self.queue = Queue()

  def tick(self):
    """Repeatedly call from target thread to execute pending events."""
    while True:
      try:
        event, args, kwargs = self.queue.get_nowait()
      except Empty:
        break

      for observer in self.observers.get(event, []):
        observer(*args, **kwargs)

      for any_observer in self.any_observers:
        any_observer(event, args, kwargs)

  def observe(self, event, callback):
    """Add callback to be triggered on the given event."""
    self.observers.setdefault(event, []).append(callback)

  def observe_any(self, callback):
    """Add callback to be triggered on any event. Invoked with 3 parameters: event, args, kwargs."""
    self.any_observers.append(callback)

  def trigger(self, event, *args, **kwargs):
    """Trigger the event with parameters."""
    self.queue.put((event, args, kwargs))
