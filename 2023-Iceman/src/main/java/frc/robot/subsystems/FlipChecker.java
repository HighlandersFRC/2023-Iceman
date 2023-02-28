// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class FlipChecker extends SubsystemBase {
  /** Creates a new FlipChecker. */

  private boolean flip = false;

  public FlipChecker() {}

  public void setFlip(Boolean flip) {
    this.flip = flip;
  }

  public boolean getFlip() {
    return flip;
  }

  @Override
  public void periodic() {
    if(OI.operatorController.getLeftBumper()) {
      setFlip(true);
    }
    else {
      setFlip(false);
    }
    // This method will be called once per scheduler run
  }
}
