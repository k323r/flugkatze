#!/usr/bin/python

import curses
from time import sleep as sleep

## init curses
screen = curses.initscr()
## enable noecho mode of the terminal
curses.noecho()
## turn of bufferin -> enter is not required to send a key stroke
curses.cbreak()
## enable curses to handle special characters
screen.keypad(1)
#################################################

pad = curses.newpad(100, 100)
#  These loops fill the pad with letters; this is
# explained in the next section
for y in range(0, 100):
    for x in range(0, 100):
        try:
            pad.addch(y,x, ord('a') + (x*x+y*y) % 26)
        except curses.error:
            pass

#  Displays a section of the pad in the middle of the screen
pad.refresh(0,0, 5,5, 20,75)

sleep(3)

#################################################
#### upon quitting: reverse the console settings
curses.nocbreak(); screen.keypad(0); curses.echo()
## end the application
curses.endwin()

