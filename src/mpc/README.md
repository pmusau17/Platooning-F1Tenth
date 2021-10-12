# MPC 

Documentation loading...(I promise)

### Installation of MPC Libraries

``
$ source setup.sh

install [HSL](https://www.do-mpc.com/en/latest/installation.html). Find the instructions in the aforementioned link, you will need to get a license to download the software.
``

### Running the mpc setup:

- mpc_model:=0 (MPC with Follow the Gap)
- mpc_model:=1 (MPC with Pure Pursuit)
- mpc_model:=2 (MPC with Pure Pursuit Nonlinear Model)

```
$ roslaunch mpc mpc.launch mpc_model:=0
```