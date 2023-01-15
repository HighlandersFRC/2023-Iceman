// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.Dictionary;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AlignToConeHorizontal;
import frc.robot.commands.AlignToConeVertical;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.MoveToReflectiveTape;
import frc.robot.commands.RunIntake;
import frc.robot.commands.VisionAlignment;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.commands.defaults.MoveArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MorrisArm;
import frc.robot.subsystems.Peripherals;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals(lights);
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private MorrisArm arm = new MorrisArm();

  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  @Override
  public void robotInit() {
    drive.init();
    peripherals.init();
    lights.init();
    arm.init();
    intake.init();
    try {
      pathingFile = new File("/home/lvuser/deploy/Test.json");
      FileReader scanner = new FileReader(pathingFile);
      // pathRead = new JSONTokener(scanner);
      pathRead = new JSONObject(new JSONTokener(scanner));
      pathJSON = (JSONArray) pathRead.get("sampled_points");
      JSONArray first = pathJSON.getJSONArray(0);
      double firstTime = first.getDouble(0);
      System.out.println(first);
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(peripherals.getCameraBasedRobotLocation());
    // peripherals.getLimelightBasedPosition();
    SmartDashboard.putNumber("latency", peripherals.getCameraLatency());
    SmartDashboard.putNumber("NAVX Roll", peripherals.getNavxRoll());
    // SmartDashboard.putBoolean("HAS TARGET", peripherals.cameraHasTargets());
    // SmartDashboard.putNumber("Target Yaw", peripherals.cameraYawToTarget());
    // SmartDashboard.putNumber("Target Pitch", peripherals.cameraPitchToTarget());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    drive.autoInit(pathJSON);
    AutonomousFollower auto = new AutonomousFollower(drive, pathJSON);

    auto.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.teleopInit();
    OI.driverViewButton.whileHeld(new ZeroNavxMidMatch(drive));
    // OI.driverRT.whileActiveOnce(new RunIntake(intake, 0.2));
    // OI.driverA.whenHeld(new MoveArm(arm, 0.25));
    // OI.driverB.whenHeld(new MoveArm(arm, -0.25));
    // OI.driverX.whenPressed(new AlignToConeHorizontal(drive, peripherals));
    // OI.driverY.whenPressed(new AlignToConeVertical(drive, peripherals));
    // OI.driverY.whenPressed(new MoveToReflectiveTape(drive, peripherals));
    OI.driverA.whileHeld(new RunIntake(intake, 0.3));
    OI.driverB.whileHeld(new RunIntake(intake, -0.3));
    OI.driverY.whileHeld(new RunIntake(intake, 0.1));
    OI.driverX.whileHeld(new RunIntake(intake, -0.1));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("NAVX", peripherals.getNavxAngle());
    // System.out.println(Constants.testTagToRobot());
    // System.out.println(Constants.calculateCameraBasedPosition());
    // System.out.println(peripherals.cameraToTarget());
    // SmartDashboard.putNumber("X", peripherals.cameraToTarget().getX());
    // SmartDashboard.putNumber("Y", peripherals.cameraToTarget().getY());
    // System.out.println(Constants.testDistanceToTagZero());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
