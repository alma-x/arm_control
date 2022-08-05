# arm_control
planning and control for robotic arm


TODO:

ENV EXPLORATION:
- stop in specific positions and aquire markers only when not moving
- set ground truth aruco reference positions to compute the error w/ the current implementation
- 

COLLISION OBJECTS (MEANINGFUL ARUCOs):
- table/base plane (14,(10))
- frame structure/robot base (fixed)
- mid panel (1-9)
- [1-9] buttons (i in 1-9)
{- [1-9] aruco+button frames  (i in 1-9)}
- left panel (11)
{- aruco frame (11)}
- imu module (10)
- right panel (12,13)
- lid (13)
- inspection box (12,13)


ERRORS
- gripper command fails when requests are too close: should wait... actions?