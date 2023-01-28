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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RotateArm;
import frc.robot.commands.RunWrist;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.commands.TwoPieceAuto;
import frc.robot.commands.VisionAlignment;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

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
  private Arm arm = new Arm();
  private Wrist wrist = new Wrist();

  private UsbCamera frontDriverCam;
  private UsbCamera backDriverCam;

  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  File pathingFile2;
  String pathString2;

  JSONObject pathRead2;
  JSONArray pathJSON2;
  String fieldSide;

  @Override
  public void robotInit() {
    // frontDriverCam = CameraServer.startAutomaticCapture("Front Cam", "/dev/video0");
    // frontDriverCam.setResolution(320, 240);
    // frontDriverCam.setFPS(15);

    // backDriverCam = CameraServer.startAutomaticCapture("Back Cam", "/dev/video1");
    // backDriverCam.setResolution(320, 240);
    // backDriverCam.setFPS(15);

    drive.init();
    peripherals.init();
    lights.init();
    arm.init();
    wrist.init();
    try {
      pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
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

    try {
      pathingFile2 = new File("/home/lvuser/deploy/2PiecePart2.json");
      FileReader scanner2 = new FileReader(pathingFile2);
      // pathRead = new JSONTokener(scanner);
      pathRead2 = new JSONObject(new JSONTokener(scanner2));
      pathJSON2 = (JSONArray) pathRead2.get("sampled_points");
    }
    catch(Exception e) {
      System.out.println("ERROR WITH PATH FILE " + e);
    }
  }  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Extension", arm.getExtensionPosition());
    SmartDashboard.putBoolean("ARM LIMIT SWITCH", arm.getExtensionLimitSwitch());
    // SmartDashboard.putNumber("ARM ROTATION", arm.getRotationPosition());

    arm.postRotationValues();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    drive.autoInit(pathJSON);

    TwoPieceAuto auto = new TwoPieceAuto(drive, arm);

    auto.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.teleopInit(); 
    OI.driverViewButton.whileTrue(new ZeroNavxMidMatch(drive));
    
    OI.driverY.whileTrue(new SetArmRotationPosition(arm, 45));
    // OI.driverX.whileTrue(new RotateArm(arm, 0.5));
    OI.driverX.whileTrue(new SetArmRotationPosition(arm, 0));
    OI.driverA.whileTrue(new ExtendArm(arm, -0.2));
    // OI.driverA.onTrue(new AutoBalance(drive));
    OI.driverB.whileTrue(new SetArmRotationPosition(arm, 180));
    // OI.driverB.whileTrue(new RotateArm(arm, -0.5));
    // OI.driverY.whenPressed(new SetArmExtensionPosition(arm, 10));

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
