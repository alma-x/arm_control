# arm_control
planning and control for robotic arm


TODO:

ENV EXPLORATION:
- currently: controller stops the robot and grants permission for referencing, but sometimes this does not work, even if all markers are found. Should give the ability to stop the robot to the detector or
change inquirer_aruco into this controller?

- {SIMULATION} set ground truth aruco reference positions to compute the error w/ the current implementation: AS FOR NOW, ESTIMATED MANUALLY

GRIPPER CONTROL
- since gripper control point TCP not defined, gripper_controller should broadcast
     it's position too.
    variying position foreach state [measured from gripper's end plate]:
        • open: TCP 151.5 mm, Aperture 78 mm 
        • semi open: TCP 186 mm, Aperture 50 mm
        • semi closed: TCP 192.5 mm, Aperture 35 mm
        • closed: TCP 200 mm, Aperture 0 mm

- refuse a request for the state gripper is already in (& suppress printing)

COLLISION INTERFACE:
- add hitbox for gripper's cable block


ERRORS----------------------------------------

MOVEIT
- gripper command fails when requests are too close (moves from expected position): should wait... actions?


CONTROLLER
- when references are already published but node restarts

[ERROR] [1659864098.022504488, 3823.495000000]: Ignoring transform for child_frame_id "..." from authority "unknown_publisher" because of an invalid quaternion in the transform (0.000000 0.000000 0.000000 0.000000)

...={robot_frame, button_3, button_4}
