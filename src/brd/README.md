# brd directory

This directory contains the board definitions.

Include a board definion with `#define MF_BOARD "brd\madflight_FC3.h"`

NOTE: Using this indirect #include allows using this library with Arduino IDE, but has the side effect that with PlatformIO IDE changes in the brd files are not detected and need a clean or a change in main.cpp