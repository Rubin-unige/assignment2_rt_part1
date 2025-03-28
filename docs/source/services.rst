Services
========

This section documents the custom ROS services defined in this package.

get_last_service.srv
--------------------
This service returns the last target coordinates (x, y) that were sent to the robot.

**Request**
    - The request is empty (no fields).

**Response**
    - **x** (float64): The last target x-coordinate.
    - **y** (float64): The last target y-coordinate.

Service Definition
------------------
.. code-block:: text

    # Request
    ---
    # Response
    float64 x  # The last target x-coordinate
    float64 y  # The last target y-coordinate