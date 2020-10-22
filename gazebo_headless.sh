#/bin/bash
Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0

