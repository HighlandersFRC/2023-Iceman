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
import frc.robot.commands.AutoPlacementCone;
import frc.robot.commands.AutoPlacementCube;
import frc.robot.commands.AlignToConePlacement;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonomousFollower;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.MoveToPiece;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.RotateArm;
import frc.robot.commands.RunWrist;
import frc.robot.commands.SetArmExtensionPosition;
import frc.robot.commands.SetArmRotationPosition;
import frc.robot.commands.VisionAlignment;
import frc.robot.commands.ZeroNavxMidMatch;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Drive;
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
  }  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putBoolean("IS CLEAR SIDe", OI.isClearSideAuto());

    SmartDashboard.putNumber("NAVX", peripherals.getNavxAngle());

    SmartDashboard.putNumber("Extension", armExtension.getExtensionPosition());
    SmartDashboard.putBoolean("ARM LIMIT SWITCH", armExtension.getExtensionLimitSwitch());

    armRotation.postRotationValues();
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
      // System.out.println("GOT RED");
    }
    else if(OI.isBlueSide()) {
      drive.setFieldSide("blue");
      // System.out.println("GOT BLUE");
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

    // OI.driverA.onTrue(new AutoPlacementCube(drive, peripherals, lights, 0));
    // OI.driverX.onTrue(new AutoPlacementCone(drive, peripherals, lights, -1));
    // OI.driverB.onTrue(new AutoPlacementCone(drive, peripherals, lights, 1));

    OI.driverA.whileHeld(new RunWrist(wrist, -1, -1));
    OI.driverY.whileHeld(new RunWrist(wrist, 1, -1));

    OI.driverX.whileActiveOnce(new MoveToPiece(drive, peripherals, lights));

    OI.operatorA.whenPressed(new SetArmExtensionPosition(armExtension, 1));
    OI.operatorB.whileHeld(new SetArmExtensionPosition(armExtension, 20));
    OI.operatorX.whileHeld(new SetArmExtensionPosition(armExtension, 35));
    OI.operatorY.whenPressed(new SetArmRotationPosition(armRotation, 180));
    OI.operatorLB.whileHeld(new SetArmRotationPosition(armRotation, 90));
    OI.operatorRB.whileHeld(new SetArmRotationPosition(armRotation, 125));

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
