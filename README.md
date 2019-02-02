[![Build Status](https://travis-ci.com/iron-claw-972/FRC2019.svg?branch=master)](https://travis-ci.com/iron-claw-972/FRC2019)
[![Coverage Status](https://coveralls.io/repos/github/iron-claw-972/FRC2019/badge.svg?branch=master)](https://coveralls.io/github/iron-claw-972/FRC2019?branch=master)
[![BCH compliance](https://bettercodehub.com/edge/badge/iron-claw-972/FRC2019?branch=master)](https://bettercodehub.com/)

# FRC2019
___
### TODO
- [x] Fix code building
- [ ] Refactor Code Structure
- [x] Add teleop logic for mecanum drive (Jody)
- [ ] Add teleop logic for arcade drive (Val)
- [ ] Detect alignment lines
- [ ] Use alignment line to calculate relative position
- [ ] Setup DriverStation logging
- [ ] Send metrics to ShuffleBoard
- [ ] Use camera to detect hatch panel
- [ ] Optimize camera network
- [ ] Test camera switching on the robot (Maciej)
- [ ] Add overlay to the feed from the camera (Maciej)
- [ ] Robot localization

### Subsystems TODO
- [ ] Elevator (Daniel, Alan)
- [ ] Arm (Ethan, Pranav)
- [ ] Piston Climb (Maxim, Nikita)
- [ ] Mecanum Cargo Intake (Julian, Hope)
- [ ] Hatch Intake (Alexis, Vincent)
- [ ] Vision Localization (Maciej)
- [ ] State Space (Jody, Val)


### Set-Up Environment
- If you don't have JDK11, install from u.nu/claw
- JDK in folder `C:\Program Files\Java\jdk-11.0.1`
- Open gitbash. Run `git init`, `git clone https://github.com/iron-claw-972/FRC2019.git`
- Open intellij, select the stuff you downloaded. Go to build and run it, select jdk11 from program files if neccesary.


### Code Help
#### Joysticks:

In your subsystem:
```
ControlBoard.getThrottle();
```

Inside ControlBoard:
```
public double getThrottle() {
  return mDriveControlBoard.getTranslateY();
}
```

Inside GamepadDriveControlBoard:
```
public double getTranslateY() {
  return -mJoystick.getRawAxis(1);
}
```
#### Talons and sensors:

```
talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
talon.getSensorCollection().getQuadraturePosition();
talon.getSensorCollection().getQuadratureVelocity();
```
