// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class FlipChecker extends SubsystemBase {
  /** Creates a new FlipChecker. */

  private boolean flip = false;
  private boolean isTeleop = false;

  private boolean allowedToFlip = true;

  private Peripherals peripherals;

  public FlipChecker(Peripherals peripherals) {
    this.peripherals = peripherals;
  }

  public void setTeleop() {
    isTeleop = true;
  }

  public void setFlip(Boolean flip) {
    this.flip = flip;
  }

  public boolean getFlip() {
    return flip;
  }

  public void setAllowedToFlip(boolean flip) {
    allowedToFlip = flip;
  }

  private double getTrueDegree(double angle) {
    // converts angle to 0-360 range
    angle = angle % 360;
    if (angle < 0) {
      angle += 360;
    }
    return angle;
  }
  public boolean getArmDirection(){
    //gets angle of controller's left stick
    double joy = joystickToAngle(OI.operatorController.getLeftX(), OI.operatorController.getLeftY());
    //converts navxAngle to a 0-360 degree angle
    double nav = getTrueDegree(peripherals.getNavxAngle());
    //sets upper bound and lower bound
    double upperBound = nav + 90;
    double lowerBound = nav - 90;
    //converts bounds to a 0-360 degree angle
    upperBound = getTrueDegree(upperBound);
    lowerBound = getTrueDegree(lowerBound);
    if (upperBound < lowerBound) {
    //if the upper bound ends up lower than the lower bound, then the joystick should not be between them to return true
      return joy <= upperBound || joy >= lowerBound;
    }else{
    //if the upper bound is higher than the lower bound, then the joystick needs to be between them to return true
      return joy <= upperBound && joy >= lowerBound;
    }
  }
  private double joystickToAngle(double x, double y){
    //returns the angle of the joystick in degrees
    y *= -1;
    double angle =Math.atan2(y, x);
    angle = Math.toDegrees(angle);
    angle += 270;
    angle %= 360;
    return angle;
  }

  @Override
  public void periodic() {
    // if(OI.operatorController.getLeftBumper()) {
    //   setFlip(true);
    // }
    // else {
    //   setFlip(false);
    // }

    if(isTeleop && allowedToFlip) {
      setFlip(!getArmDirection());
    }
    else {
      setFlip(flip);
    }

    // System.out.println(allowedToFlip);

    // This method will be called once per scheduler run
  }
}
