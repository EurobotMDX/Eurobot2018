import curses
from drive import *

# get the curses screen window
screen = curses.initscr()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

start = md25(mode=1)

speed = 50
turnSpeed = 30

try:
    while True:
        global speed

        char = screen.getch()

        if char == ord('q'):
            break

        elif char == ord('f'):
            # global speed
            speed = speed + 10
            print("Speed: {}".format(speed))

        elif char == ord('s'):
            # global speed
            speed = speed - 10
            print("Speed: {}".format(speed))

        elif char == ord(' '):
            start.stop()

        elif char == curses.KEY_RIGHT:
            start.drive(turnSpeed, -turnSpeed)

        elif char == curses.KEY_LEFT:
            start.drive(-turnSpeed, turnSpeed)

        elif char == curses.KEY_UP:
            start.drive(speed, speed)

        elif char == curses.KEY_DOWN:
            start.drive(-speed, -speed)

finally:
    # shut down cleanly
    curses.nocbreak();
    screen.keypad(0);
    curses.echo()
    curses.endwin()
