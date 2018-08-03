"""Snake game on the raspberry pi sense hat."""

from time import perf_counter, sleep
from random import randint

from sense_hat import SenseHat

class State:
    def __init__(self, foods=2, snek=3, speed=2.0):
        self.snek = [(3, 3)]
        self.snek_dir = "right"
        self.snek_lives = True
        self.foods = []
        self.time = 0.0
        self.last_move = 0.0
        self.speed = 2.0

        assert snek >= 1, "Too short a snek!"
        for _ in range(snek - 1):
            self.grow_snek()
        for _ in range(foods):
            self.put_food()


    def put_food(self):
        pos = self.snek[0]
        # Find a random unoocupied position.
        while (
            intersects(self.snek, pos) is not None or
            intersects(self.foods, pos) is not None
        ):
            pos = randint(0, 7), randint(0, 7)

        # Place it there.
        self.foods.append(pos)

    def grow_snek(self):
        assert len(self.snek) >= 1, "Where the head go?"
        self.snek.append(self.snek[-1])

    def go_in_dir(self, direction):
        if self.snek_dir in ("up", "down") and direction in ("up", "down"):
            return

        if self.snek_dir in ("left", "right") and direction in ("left", "right"):
            return

        self.snek_dir = direction

    def move_snek(self, duration):
        # Skip moving if it's too early.
        self.time += duration
        if self.time < self.last_move + 1.0 / self.speed:
            return
        else:
            self.last_move += 1.0 / self.speed

        assert len(self.snek) >= 1, "Where the head go?"
        # Get the head of the snake and compute the next spot.
        next_spot = self.snek[0]

        if self.snek_dir == "right":
            mx, my = +1, 0
        elif self.snek_dir == "left":
            mx, my = -1, 0
        elif self.snek_dir == "up":
            mx, my = 0, -1
        elif self.snek_dir == "down":
            mx, my = 0, +1
        else:
            raise Exception("Weird direction {}".format(self.snek_dir))

        next_spot = (next_spot[0] + mx) % 8, (next_spot[1] + my) % 8


        if intersects(self.snek, next_spot) is not None:
            self.snek_lives = False
            raise DeadSnek("Ouch!")

        food_to_eat = intersects(self.foods, next_spot)
        if food_to_eat is not None:
            # Wooo!
            self.grow_snek()
            self.speed += 0.1
            del self.foods[food_to_eat]
            self.put_food()

        for i in range(len(self.snek)):
            # Current section already on the next spot, means we reached the end of
            # the snake and the tail is growing.
            if next_spot == self.snek[i]:
                break
            # Put current section on the next spot, and set the next spot to this
            # section.
            self.snek[i], next_spot = next_spot, self.snek[i]


def draw(state):
    # print(state.foods, state.snek)
    leds = [(0, 0, 0)] * 64
    for x, y in state.foods:
        leds[x + 8*y] = (0, 255, 0)

    hx, hy = state.snek[0]
    leds[hx + 8*hy] = (0, 0, 255) if state.snek_lives else (255, 0, 0)

    for x, y in state.snek[1:]:
        leds[x + 8*y] = (255, 255, 255)

    sensey.set_pixels(leds)

def check_input():
    events = sensey.stick.get_events()

    if any(e.action == "pressed" and e.direction == "middle" for e in events):
        return "action"

    directions = ("up", "down", "left", "right")

    # Take the last event.
    for e in reversed(events):
        if e.action == "pressed" and e.direction in directions:
            return e.direction

    return None

def game():
    state = State()
    status = "running"

    draw(state)
    while True:
        duration = yield

        command = check_input()
        if command is not None:
            if command == "action":
                if status == "running":
                    status = "paused"
                elif status == "paused":
                    status = "running"
                elif status == "paused-dead":
                    state = State()
                    status = "running"
                else:
                    raise Exception("Weird status: {}".format(status))
            elif command in ("left", "right", "up", "down"):
                state.go_in_dir(command)
            else:
                raise Exception("Weird command: {}".format(command))

        if status == "running":
            try:
                state.move_snek(duration)
            except DeadSnek:
                status = "paused-dead"
                score = len(state.snek)
                sensey.show_message(
                    "Snek was {} long. It's like this much O".format(score) + "="*(score-1))

        draw(state)


sensey = SenseHat()
def main():
    sensey.low_light = True
    timed_loop(game(), fps=30)

class DeadSnek(Exception):
    pass

def intersects(what, pos):
    for i, ref_pos in enumerate(what):
        if pos == ref_pos:
            return i
    return None

def timed_loop(genfunc, fps=60):
    duration = 1.0 / fps

    # Boot it
    last = perf_counter()
    genfunc.send(None)

    while True:
        step_duration = perf_counter() - last

        if step_duration < duration:
            sleep(duration - step_duration)

        last += step_duration

        try:
            genfunc.send(step_duration)
        except (StopIteration, KeyboardInterrupt):
            print("Game's over, you can go home and play now!")
            return


if __name__ == "__main__":
    main()
