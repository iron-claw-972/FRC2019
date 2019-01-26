# FRC2019
___
### TODO
- [ ] Fix code building
- [ ] Refactor Code Structure
- [x] Add teleop logic for mecanum drive (Jody)
- [ ] Add teleop logic for arcade drive (Val)
- [ ] Detect alignment lines
- [ ] Test IR vision with alignment line
- [ ] Use alignment line to calculate relative position
- [ ] Setup DriverStation logging
- [ ] Send metrics to ShuffleBoard
- [ ] Use camera to detect hatch panel
- [ ] Optimize camera network
- [ ] Test camera switching on the robot (Maciej)
- [ ] Add overlay to the feed from the camera (Maciej)
- [ ] Robot localization

### Set-Up Environment
- If you don't have JDK11, install from u.nu/claw
- JDK in folder `C:\Program Files\Java\jdk-11.0.1`
- Open gitbash. Run `git init`, `git clone https://github.com/iron-claw-972/FRC2019.git`
- Open intellij, select the stuff you downloaded. Go to build and run it, select jdk11 from program files if neccesary.

### Code Help
##### Joysticks
In your subsystem:
'''ControlBoard.getThrottle();
'''
Inside ControlBoard:
'''public double getThrottle() {
  return mDriveControlBoard.getTranslateY();
}
'''
Inside GamepadDriveControlBoard:
'''public double getTranslateY() {
  return -mJoystick.getRawAxis(1);
}
'''
