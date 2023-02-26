package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Wrist;

public class RotateWrist extends CommandBase {
  /** Creates a new RunIntake. */
  private Wrist wrist;
  private double angle;
  private FlipChecker flipChecker;

  public RotateWrist(Wrist wrist, FlipChecker flipChecker, double angle) {
    this.wrist = wrist;
    this.angle = angle;
    this.flipChecker = flipChecker;
    addRequirements(this.wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setAngle = angle;
    // upright cone and check to see which side to intake on
    if(angle == Constants.UPRIGHT_CONE_WRIST_ROTATION) {
      if(flipChecker.getFlip()) {
        setAngle = Constants.UPRIGHT_CONE_FLIPPED_WRIST_ROTATION;
      }
    }
    // tipped over cone and check to see which side to intake on
    else if(angle == Constants.TIPPED_CONE_WRIST_ROTATION) {
      if(flipChecker.getFlip()) {
        setAngle = Constants.TIPPED_CONE_FLIPPED_WRIST_ROTATION;
      }
    }
    else if(angle == Constants.HIGH_PLACEMENT_WRIST_ROTATION) {
      if(flipChecker.getFlip()) {
        setAngle = Constants.HIGH_PLACEMENT_FLIPPED_WRIST_ROTATION;
      }
    }
    else if(angle == Constants.MID_PLACEMENT_WRIST_ROTATION) {
      if(flipChecker.getFlip()) {
        setAngle = Constants.MID_PLACEMENT_FLIPPED_WRIST_ROTATION;
      }
    }
    if(Math.abs(OI.operatorController.getRawAxis(5)) > 0.1) {
      this.wrist.setRotationMotorPercent(OI.operatorController.getRawAxis(5)/4);
    }
    else {
      this.wrist.setWristRotationPosition(setAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(wrist.getAdustedWristRotation());
    if(Math.abs(wrist.getAdustedWristRotation() - angle) < 3) {
      return true;
    }
    return false;
  }
}
