# Lab 3: Wall Following

**Assignment Link:** https://github.com/f1tenth/f1tenth_lab3_template

**Deliverable":** Implementation of PID control through a node and a video submission of the simulation proving it works

![Intuitive process map of the code](image.png)

**Blockers before sim testing:**
- Node file was initially under `scripts` folder, so the command to run it didn't work (ROS couldn't find it)
- There were errors in the code, but they weren't printing out unless you run the command to run script directly in terminal

**Sim test tuning:**
- sharp and slow turns -> increase kd
- update velo logic to be more smooth
- "double pump" before turning -> we are adjust too fast bc kd is too high 
- falling for the divot (false turning) -> update get_range function to use "cone of vision" instead
- change desired dist (from left wall) to 1.0
