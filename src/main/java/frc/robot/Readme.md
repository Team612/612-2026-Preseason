# Readme

The directions below describe physical stick movement. Controller axis values are inverted in the robot code to match the intended drive directions.

## Robot Relative Behaviours

Robot-relative driving commands translation relative to the robot's current orientation and does not use gyro heading for translation.
- Push Left Joystick Y forward: moves the robot forward (+robot X); pull back: moves backward.
- Push Left Joystick X left: strafes left (+robot Y); push right: strafes right.
- Push Right Joystick X right: rotates clockwise; push left: rotates counterclockwise.

## Field Relative Behaviours

Field-relative driving uses WPILib's field-relative chassis-speed transform and the Pigeon2 yaw as the robot heading. Translation stays aligned to the heading-zero field axes as the robot turns. There is no automatic alliance or driver-station perspective adjustment.
- Push Left Joystick Y forward: moves toward field +X; pull back: moves toward field -X.
- Push Left Joystick X left: moves toward field +Y; push right: moves toward field -Y.
- Right Joystick X controls rotation with the same clockwise/counterclockwise directions as robot-relative mode.

The robot starts in field-relative mode. The right bumper toggles between field-relative and robot-relative modes when the sticks are centered and the robot is stopped. A valid Pigeon heading is required to drive field-relative.

Press Start to set the current Pigeon heading to zero; the current robot orientation then defines field +X. The sticks must be centered and the robot must be stopped.
