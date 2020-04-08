#/bin/bash
#Xvfb :1 -screen 0 1024x768x24  &
#Xvfb :1 -screen 0 800x600x16 &
Xvfb :1 -screen 0 1600x1200x16 &
#export DISPLAY=:1.0
export DISPLAY=:1.0
echo "Done"

