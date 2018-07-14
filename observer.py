"""Implement the observer pattern for inter thread communication."""

from queue import Queue, Empty, Full

import asyncio

class Dispatcher:
  """Event dispatcher."""

  def __init__(self, maxsize=0):
    """Dispatch up to `maxsize` events, events that are not observed in time are dropped."""
    self.queue = Queue(maxsize=maxsize)

  def trigger(self, event, data=None):
    """Trigger the `event` with `data`."""
    try:
      self.queue.put_nowait((event, data))

    # If it was full, discard the top of the queue and try again.
    except Full:
      try:
        self.queue.get_nowait()
      except Empty:
        pass
      # If it's still full this time, something's weird.
      self.queue.put_nowait((event, data))

  __call__ = trigger


class Observable:
  """Event listener."""

  def __init__(self, dispatcher):
    """Observe events from dispatcher; should be initialized from `observable`."""
    self.__observers = {}
    self.__any_observers = []
    self.__queue = dispatcher.queue


  def __try_to_observe_event(self):
    """Try to get 1 event from the queue and pass it to observers."""
    try:
      event, data = self.__queue.get_nowait()
    except Empty:
      return None

    for observer in self.__observers.get(event, []):
      observer(data)

    for any_observer in self.__any_observers:
      any_observer(event, data)

    return (event, data)


  def tick(self):
    """Repeatedly call to observe all pending events."""
    while self.__try_to_observe_event():
      pass


  def on(self, event, callback):
    """Add callback to be triggered on the `event`. Invoked witha single parameter: `data`."""
    self.__observers.setdefault(event, []).append(callback)


  def on_any(self, callback):
    """Add callback to be triggered on any event. Invoked with 2 parameters: event, data."""
    self.__any_observers.append(callback)


  async def __aiter__(self):
    """ Iterate asynchornously over all events."""
    while True:
      event_tuple = self.__try_to_observe_event()
      if event_tuple is not None:
        yield event_tuple
      else:
        await asyncio.sleep(0.0)




def create_observable(maxsize=0):
  """Create `Observable`, `Dispatcher` pair for event routing."""
  dispatcher = Dispatcher(maxsize=maxsize)
  observable = Observable(dispatcher)

  return observable, dispatcher
