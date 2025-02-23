Launch
======

This section provides instructions for launching the necessary nodes and the simulation environment for the `assignment2_rt_part1` package.

1. **Launch the `assignment_2_2024` Package**
   
   Before launching your nodes, ensure that the simulation environment is running. To start the simulation in Gazebo and RViz, run the following command in your terminal:

   .. code-block:: bash

      roslaunch assignment_2_2024 assignment1.launch

   This command will spawn the robot in the simulation environment and start the action server, which the Action Client Node will interact with. 
   
.. note:: Please wait for everything to load properly before proceeding to the next step.

2. **Launch the Action Client Node and Service Node**

   After the simulation environment is up and running, you can launch the Action Client Node and the Service Node. To do this, execute the following command:

   .. code-block:: bash

      roslaunch assignment2_rt_part1 coordinate_control.launch

   This will start the Python version of both the action client and the service node, allowing them to communicate and function as intended.
