Message
========

This section documents the custom ROS messages defined in this package.

robot_status.msg
----------------

This message contains the robot's current position and velocity.

- **x** (float64): The current x-coordinate of the robot.
- **y** (float64): The current y-coordinate of the robot.
- **vel_x** (float64): The linear velocity of the robot in the x-direction.
- **vel_z** (float64): The angular velocity of the robot around the z-axis.

Message Definition
------------------

.. code-block:: text

    # Current x-coordinate of the robot
    float64 x

    # Current y-coordinate of the robot
    float64 y

    # Linear velocity of the robot in the x-direction
    float64 vel_x

    # Angular velocity of the robot around the z-axis
    float64 vel_z