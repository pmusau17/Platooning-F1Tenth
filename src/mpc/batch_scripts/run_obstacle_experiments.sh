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

# Change this to toggle obstacles

enable_static_obstacles=false
enable_dynamic_obstacles=True

# how many cars to run 
number_of_cars=2

# Change this to change the opponent driving model
# 0 is the disparity extender
# 1 is pure pursuit keep disparity extender if you want the opponent to have 
# obstacle avoidance capabilities
opponent_model=0

# Worlds 
# 0 track_porto
# 1 racecar_walker
# 2 track_barca
worlds="0"

# Methods
# 0 is mpc with the hyper-plane formulation
# 1 is the mpcc formulation
# 2 is disparity extender
# 3 is pure pursuit

methods="0 1 3 2 "

# Target Models
# 0 is the disparity extender
# 1 is pure pursuit
target_models="1 0"


for world_number in $worlds
do 
    for mpc_model in $methods
    do
        for target_model in $target_models
        do
            # timeout 
            timeout=700

            # ignore this
            exit_status=0

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
            roslaunch mpc mpc_batch.launch mpc_model:=$mpc_model target_model:=$target_model \
            opponent_model:=$opponent_model timeout:=$timeout \
            enable_dynamic_obstacles:=$enable_dynamic_obstacles \
            enable_static_obstacles:=$enable_static_obstacles \
            world_number:=$world_number \
            number_of_cars:=$number_of_cars \
            experiment_number:=$count &


                
            child=$!
            wait "$child"
            if [ $exit_status -eq 130 ]; then
                # SIGINT was captured meaning the user
                # wants full stop instead of start_simulation.launch
                # terminating normally from end of episode so...
                echo "stop looping"
                exit 0 
            fi
            echo count: $count
            done
         done
   done
done 