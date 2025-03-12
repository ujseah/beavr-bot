<p align="center">
  <img src="docs/images/beavr_logo.png" alt="BeaVR-Bot Logo" width="300"/>
</p>

# BeaVR-Bot

## Bimanual, multi-Embodiment, Accessible, Virtual Reality Teleoperation System for Robots

This is the official implementation of BeaVR teleoperation control pipeline.

BeaVR consists of two parts. 

- Teleoperation using Meta Quest 3 and data collection over a range of robot morphologies and simulation environments.

- Policy training for various dexterous manipulation tasks across different robots and simulations.

### Server Code Installation 

Install the conda environment from the yaml file in the codebase

**Allegro Sim**

`conda env create -f env_isaac.yml`

**Others**

`conda env create -f environment.yml`

This will install all the dependencies required for the server code.  

After installing all the prerequisites, you can install this pipeline as a package with pip:

`pip install -e . `

You can test if it had installed correctly by running `import beavr` from the python shell.
