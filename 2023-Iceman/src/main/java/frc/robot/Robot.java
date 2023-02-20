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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.AutoPlacementCone;
import frc.robot.commands.AutoPlacementCube;
import frc.robot.commands.AlignToConePlacement;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.MoveToPieceBackwards;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.RotateArm;
import frc.robot.commands.RotateWrist;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.commands.SetLightMode;
import frc.robot.commands.VisionAlignment;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

import frc.robot.commands.autos.TwoPieceAuto;
import frc.robot.commands.autos.TwoPieceBumpAuto;

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
  private ArmExtension armExtension = new ArmExtension();
  private ArmRotation armRotation = new ArmRotation(armExtension);
  private Wrist wrist = new Wrist();
  private Intake intake = new Intake();

  private UsbCamera frontDriverCam;
  private UsbCamera backDriverCam;

  File pathingFile;
  String pathString;

  JSONObject pathRead;
  JSONArray pathJSON;

  String fieldSide;

  @Override
  public void robotInit() {
    drive.init();
    peripherals.init();
    lights.init();
    armExtension.init();
    armRotation.init();
    wrist.init();
    intake.init();
  }  

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("WRIST POSITION", wrist.getWristRotationPosition());
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Extension", armExtension.getExtensionPosition());
    SmartDashboard.putBoolean("ARM LIMIT SWITCH", armExtension.getExtensionLimitSwitch());

    armRotation.postRotationValues();
    // System.out.println(armRotation.getRotationPosition());
    System.out.println((wrist.getWristRotationPosition()) - 180 + 13);
    // SmartDashboard.putNumber("WRIST ROTATION", (wrist.getWristRotationPosition()) - 180 + 13);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if(OI.isBumpSideAuto()) {
      System.out.println("||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
      try {
        pathingFile = new File("/home/lvuser/deploy/2PieceBumpPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.isClearSideAuto()) {
      System.out.println("------------------------------------------------------------");
      try {
        pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      }
      catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    if(OI.isRedSide()) {
      drive.setFieldSide("red");
      lights.setFieldSide("red");
    }
    else if(OI.isBlueSide()) {
      drive.setFieldSide("blue");
      drive.setFieldSide("blue");
    }
    
    drive.autoInit(pathJSON);

    System.out.println(peripherals.getNavxAngle());

    if(OI.isBumpSideAuto()) {
      TwoPieceBumpAuto auto = new TwoPieceBumpAuto(drive, armExtension, armRotation, wrist, peripherals, lights);
      auto.schedule();
    }
    else if(OI.isClearSideAuto()) {
      TwoPieceAuto auto = new TwoPieceAuto(drive, armExtension, armRotation, wrist, peripherals, lights);
      auto.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.teleopInit(); 
    OI.driverViewButton.whileTrue(new ZeroNavxMidMatch(drive));

    OI.operatorA.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 221), new RotateWrist(wrist, -123), new SetArmExtensionPosition(lights, armExtension, armRotation, 14)));
    OI.operatorY.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 221), new RotateWrist(wrist, -127), new SetArmExtensionPosition(lights, armExtension, armRotation, 37)));


    // OI.operatorB.whileHeld(new SetArmExtensionPosition(armExtension, armRotation, 14));
    // OI.operatorX.whileHeld(new SetArmExtensionPosition(armExtension, armRotation, 30));

    // intake positions for upright cone
    OI.operatorX.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 87.5), new RotateWrist(wrist, 61)));
    OI.operatorB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 269.5), new RotateWrist(wrist, -63)));

    OI.operatorRB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 291.5), new RotateWrist(wrist, 3)));
    OI.operatorLB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 69.5), new RotateWrist(wrist, -3)));

    // OI.operatorA.whileHeld(new RunIntake(intake, 6, 1));
    // OI.operatorY.whileHeld(new RunIntake(intake, -6, 1));

    OI.operatorViewButton.whenPressed(new SetLightMode(lights, "cube"));
    OI.operatorMenuButton.whenPressed(new SetLightMode(lights, "cone"));

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
