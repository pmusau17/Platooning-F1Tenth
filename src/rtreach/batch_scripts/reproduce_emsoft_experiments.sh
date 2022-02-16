#!/bin/bash


# Notes (mostly for me)
# $! Contains the process ID of the most recently executed background pipeline 
# $? This the exit status of the last executed command  
# The linux trap command allows you to catch signals and execute code when they occur
# SIGINT is generated when you type Ctrl-C at the keyboard to interrupt a running script 


# kill -INT $pid sends the "interrupt" signal to the process with process ID pid. 
# However, the process may decide to ignore the signal, or catch the signal and do 
# something before exiting and/or ignore it.


_term() {
  exit_status=$? # = 130 for SIGINT
  echo "Caught SIGINT signal!"
  kill -INT "$child" 2>/dev/null
}

# for future reference generating a random number within a range
# $(shuf -i 0-4 -n 1)

Velocities="0.5 1.0 1.5"
Runtimes="25 10"
Obstacles="0 6"
for ((algorithm_number=0;algorithm_number<4;algorithm_number++))
do 
    for vel in $Velocities
    do
        for rt in $Runtimes
        do
            for obs in $Obstacles
            do 
                count=0
                trap _term SIGINT
                while [ $count -lt 30 ]
                do
                ((count=count+1)) 
                echo "${algorithm_number}|${vel}|${rt}|${obs}|${count}"


                roslaunch race sim_for_rtreach_batch_emsoft.launch random_seed:=$count algorithm:=$algorithm_number velocity:=$vel world_number:=0 num_obstacles:=$obs wall_time:=$rt experiment_number:=$count timeout:=60 & 
                child=$!
                wait "$child"
                if [ $exit_status -eq 130 ]; then
                    # SIGINT was captured meaning the user
                    # wants full stop instead of start_simulation.launch
                    # terminating normally from end of episode so...
                echo "stop looping"
                echo count: $count
                exit 1
                fi
                
                done



            done
        done
    done 
done 
