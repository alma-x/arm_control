# arm_control
planning and control for robotic arm


TODO:

ENV EXPLORATION:
- stop in specific positions and aquire markers only when not moving
    with a request client to referencer

- set ground truth aruco reference positions to compute the error w/ the current implementation

GRIPPER CONTROL
- since gripper control point TCP not defined, gripper_controller should broadcast
     it's position too.
    variying position foreach state [measured from gripper's end plate]:
        • open: TCP 151.5 mm, Aperture 78 mm 
        • semi open: TCP 186 mm, Aperture 50 mm
        • semi closed: TCP 192.5 mm, Aperture 35 mm
        • closed: TCP 200 mm, Aperture 0 mm

ERRORS
- gripper command fails when requests are too close: should wait... actions?