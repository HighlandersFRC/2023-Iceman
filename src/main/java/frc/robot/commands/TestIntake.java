package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Intake;

public class TestIntake extends CommandBase {
  
  Intake intake;
  Lights lights;

  public TestIntake(Intake intake, Lights lights) {
    this.intake = intake;
    this.lights = lights;
    addRequirements(intake, lights);
  }

  public double time;

  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
    intake.setIntakeTorqueOutput(5, .2);
  }

  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - time > 0.2) {
      if(intake.getVelocity() < 20) {
        lights.setLightsToIntake();
      } else {
        lights.setCorrectMode();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeTorqueOutput(0, 0);
    lights.setCorrectMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
