import serial
import curses

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(1)

def switcher(key):
    switch = {
        w:
        a:
        s:
        d:
        x:
    }

while (True):
    key = stdscr.getch()     
    stdscr.refresh()    
    
    #TO DO: send key over serial
    
    
    if (key == 'x'):
        break   

curses.nocbreak()
stdscr.keypad(0)
curses.echo()
curses.endwin()


