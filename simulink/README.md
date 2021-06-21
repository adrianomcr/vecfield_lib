# vecfield_lib - simulink

Vector field control codes for simulink. There are two simulation, one that controls a fixed-wing UAV and the other that controls a quadcopter.

---------------------

### Requirements

Some plotting function of the examples use the matlab_lib. Follow the instruction in <https://github.com/adrianomcr/matlab_lib> to install it.



### Fixed-wing UAV

Simulation of a fixed wing UAV using artificial vector fields. A complex airplane model is considered and lower level PID controllers are considered.

Open the file `simulink/fixed_wing/fixed_wing_sim.slx` on simulink to run the simulation.

Related Publication:

[1] Adriano M.C. Rezende; Vinicius M. Gonçalves; Guilherme V. Raffo; Luciano C.A. Pimenta **Robust Fixed-Wing UAV Guidance with Circulating Artificial Vector Fields**, *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* DOI: 10.1109/IROS.2018.8594371



### Quadcopter

Simulation of a quadrotor using artificial vector fields. The acro mode of operation is considered, the inputs are the thrust force and the angular rates.

Open the file `simulink/quadcopter/quad_sim.slx` on simulink to run the simulation.

Related Publication:

[2] Adriano M. C. Rezende; Vinicius M. Gonçalves; Arthur H. D. Nunes; Luciano C. A. Pimenta **Robust quadcopter control with artificial vector fields**, *2020 IEEE International Conference on Robotics and Automation (ICRA)* DOI: 10.1109/ICRA40945.2020.9196605

