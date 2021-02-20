#!/bin/bash


# Notes (mostly for me)
# $! Contains the process ID of the most recently executed background pipeline 
# $? This the exit status of the last executed command  
# The linux trap command allows you to catch signals and execute code when they occur
# SIGINT is generated when you type Ctrl-C at the keyboard to interrupt a running script 


# kill -INT $pid sends the "interrupt" signal to the process with process ID pid. 
# However, the process may decide to ignore the signal, or catch the signal and do 
# something before exiting and/or ignore it.


# for future reference generating a random number within a range
# $(shuf -i 0-4 -n 1)




# Change this to evaluate on a different track 
# 0 is porto, 1 is racecar_walker, 2 is barca
world_number=2

# Change this to select the algorithm
# 0 is e2e porto training, 1 is SAC, 2 is ddpg, 3 is e2e all tracks
algorithm_number=3

# ignore this
exit_status=0

# Select the velocity for evaluation 
velocity=1.5

# this keeps track of how many experiments we have run
count=0 
_term() {
  exit_status=$? # = 130 for SIGINT
  echo "Caught SIGINT signal!"
  kill -INT "$child" 2>/dev/null
}


trap _term SIGINT

while [ $count -lt 30 ]
do
((count=count+1)) 
roslaunch race model_comparison.launch algorithm:=$algorithm_number world_number:=$world_number velocity:=$velocity experiment_number:=$count\
    timeout:=60 &
     

child=$!
wait "$child"
if [ $exit_status -eq 130 ]; then
    # SIGINT was captured meaning the user
    # wants full stop instead of start_simulation.launch
    # terminating normally from end of episode so...
    echo "stop looping"
    break
fi
echo count: $count
done
