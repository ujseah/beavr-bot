# Simulation Environment Instructions

Make sure you have installed all dependencies as described in the [main README](../README.md).

The simulation is made completely in PyBullet's simulation engine. To run the simulation, run this command:

`python3 teleop.py robot=leap_pybullet_xarm`

and modify this code snippet under [`teleop.yaml`](../configs/teleop.yaml):

```python
# To run the simulation environment
sim_env: true
```

As this page may be the first contact you have with this code, for Quest installation, use manual, and IP address issues, please follow [`VR guidelines`](vr.md).
> **Note:** The initializer of this project will parse the exact name of the configuration file (without the .yaml extension) to fetch the configuration. Remember to match the name of the config file, such as leap_pybullet_xarm.yaml

You will see a broadcasted video that looks like this:

<!-- TODO: Insert gif here -->

To run the arm or hand separately, use the names `leap_pybullet` or `xarm_pybullet_sim`.