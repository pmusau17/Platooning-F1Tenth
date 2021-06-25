echo "In runxvfb"
Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0
echo $DISPLAY
echo "done runxvfb"
