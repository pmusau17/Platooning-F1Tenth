# Reinforcement Learning 
These methods are designed for training a vehicle follower, which we plan on expanding to platooning. However, we might 
expand this to train a single racer in the future.

Both methods require a parameter file is loaded before running. We'll explain these more below, but they contain all of 
hyperparameters for the learning algorithm. Modifying these can create different results which might be better than our 
performance. We use nominal values discussed in papers opting for consistent results instead of peak performance.

### DDPG
Deep Deterministic Policy Gradient (DDPG) is explained in [Lillicrap et. al, 2015](https://arxiv.org/abs/1509.02971).
The method uses model-free, off-policy learning to determine an optimal deterministic policy for the agent to follow. 
This means, during training, the agent executes random actions not determined using the learned policy. This string of 
randomly executed actions is stored in a replay buffer, which is sampled from after each step to learn an estimation of 
the state-action value (a.k.a. Q-value) function. This Q-function is used to train the policy to take actions that result in the 
largest Q-value. The folder containing this method, [DDPG](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/ddpg_control), 
is made of the following parts:
* [config_ddpg.yaml](ddpg_control/config_ddpg.yaml): 
YAML file with all of the configuration information including the hyperparameters used for training. 
* [ddpg.py](ddpg_control/ddpg.py): The main file for 
running this method. Every step of the DDPG algorithm is implemented in this file.
* [nn_ddpg.py](ddpg_control/nn_ddpg.py): The neural network 
architecture described in [Lillicrap et. al, 2015](https://arxiv.org/abs/1509.02971) is implemented here using PyTorch. 
Forward passes and initialization are handled in this class as well.
* [noise_OU.py](ddpg_control/noise_OU.py): 
Ornstein-Uhlenbeck process noise is implemented in this file. This noise is applied to create the random exploration 
actions. This is the same noise used in [Lillicrap et. al, 2015](https://arxiv.org/abs/1509.02971).
* [replay_buffer.py](ddpg_control/replay_buffer.py):

To run this method, do the normal launch of the racing environment with 2 cars, then in a separate terminal:
```bash
$ rosparam load config_ddpg.yaml
$ rosrun rl ddpg.py
```

### PPO
Proximal Policy Optimization (PPO) is explained in [Schulman et. al, 2017](https://arxiv.org/abs/1707.06347) as a simplified 
improvement to Trust Region Policy Optimization ([TRPO](https://arxiv.org/abs/1502.05477)). The method uses model-free, 
on-policy learning to determine an optimal stochastic policy for the agent to follow. This means, during training, the 
agent executes actions randomly chosen from the output policy distribution. The policy is followed over the course of a 
horizon. After the horizon is completed, the Advantages are computed and used to determine the effectiveness of the 
policy. The Advantage values are used along with the probability of the action being taken to compute a loss function, 
which is then clipped to prevent large changes in the policy. The clipped loss is used to update the policy and improve 
future Advantage estimation. The folder containing this method, [PPO](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/ppo_control), 
is made of the following scripts:
* [class_nn.py](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/ppo_control/class_nn.py): The neural network 
architecture described in [Lillicrap et. al, 2015](https://arxiv.org/abs/1509.02971) is implemented here using PyTorch. 
Forward passes and initialization are handled in this class as well. We are currently working on changing the 
architecture to match that used in [Schulman et. al, 2017](https://arxiv.org/abs/1707.06347).
* [ppo.py](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/ppo_control/ppo.py): The main file for 
running this method. Every step of the PPO algorithm is implemented in this file.
* [ppo_config.py](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/scripts/ppo_control/ppo_config.yaml): YAML file 
with all of the configuration information including the hyperparameters used for training. 

To run this method, do the normal launch of the racing environment with 2 cars, then in a separate terminal:
```bash
$ rosparam load ppo_config.yaml
$ rosrun rl ppo.py
```

# Reset the Environment 

If the car crashes or you want to start the experiment again. Simply run:

 ```bash
 $ rosrun race reset_world.py
 ```

to restart the experiment.

# Running teleoperation nodes

To run a node to tele-operate the car via the keyboard run the following in a new terminal:

```bash
$ rosrun race keyboard_gen.py racecar
```

'racecar' can be replaced with 'racecar1' 'racecar2' if there are multiple cars. 

Additionally if using the f1_tenth_devel.launch file, simply type the following:

```bash
$ roslaunch race f1_tenth_devel.launch enable_keyboard:=true