import asyncio
import logging
import traceback
import sys
import shlex


def run_tasks(*tasks, commands={}):
  """Run async tasks until completion and handle input commands.

  Commands are handled asynchornously from standard input, with the same syntax
  as unix programs. The special command `q`, `quit` or `exit` cancels all tasks.

  Args:
    tasks: Futures or coroutines to complete.
    commands: Dictionary of command, callback. The callback is invoked with the
    argv list of commands as if it was a unix program.
  """

  async def async_main():
    loop = asyncio.get_event_loop()

    # Start all tasks that we need to run.
    pending = [loop.create_task(t) for t in tasks]

    # In case of unhandled exceptions or the quit command, we cancel all
    # remaining tasks and exit.
    def cancel_pending():
      print(f"Cancelling {len(pending)} tasks!")
      for task in pending:
        task.cancel()


    # Define input handler to dispatch commands.
    def handle_input():
      line = sys.stdin.readline()
      argv = shlex.split(line)

      if not argv:
        return

      command = argv[0]

      if command in ("q", "quit", "exit"):
        cancel_pending()
        return

      try:
        func = commands[command]
      except KeyError:
        print(f"Unknown command: {command}")
        return

      try:
        coro = func(argv)
      except Exception as exc:
        print(f"Error while invoking command: {command}")
        print_exception(exc)

      task = loop.create_task(coro)

      pending.append(task)


    loop.add_reader(sys.stdin.fileno(), handle_input)

    # Run all pending tasks.
    try:
      while pending:
        done, pending = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
          try:
            await task
          except asyncio.CancelledError:
            pass
          except Exception as exc:
            print_exception(exc)
            cancel_pending()

    # Clear interrupt handlers before exiting.
    finally:
      loop.remove_reader(sys.stdin.fileno())


  # Sigh, no good way to gracefully exit on KeyboardInterrupt... because the
  # event loop logic might be interrupted and we can't nicely recover from it.
  asyncio.run(async_main())


def print_exception(exc):
  return traceback.print_exception(type(exc), exc, exc.__traceback__)

