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
import edu.wpi.first.net.PortForwarder;
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
import frc.robot.commands.MoveToPieceForwards;
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
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;
// import edu.wpi.first.util.net.PortForwarder;

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
  private FlipChecker flipChecker = new FlipChecker();

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

    PortForwarder.add(5800, "limelight-front.local", 5800);
    PortForwarder.add(5801, "limelight-front.local", 5801);
    PortForwarder.add(5805, "limelight-front.local", 5805);

    PortForwarder.add(5800, "limelight-back.local", 5800);
    PortForwarder.add(5801, "limelight-back.local", 5801);
    PortForwarder.add(5805, "limelight-back.local", 5805);
  }  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    flipChecker.periodic();

    SmartDashboard.putNumber("Extension", armExtension.getExtensionPosition());
    SmartDashboard.putBoolean("ARM LIMIT SWITCH", armExtension.getExtensionLimitSwitch());

    armRotation.postRotationValues();
    // System.out.println("ARM: " + armRotation.getRotationPosition());
    System.out.println("WRIST: " + wrist.getAdustedWristRotation());
    // System.out.println("EXTENSION: " + armExtension.getExtensionPosition());
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
      TwoPieceBumpAuto auto = new TwoPieceBumpAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
      auto.schedule();
    }
    else if(OI.isClearSideAuto()) {
      TwoPieceAuto auto = new TwoPieceAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
      auto.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.teleopInit(); 
    armExtension.teleopInit();

    OI.driverViewButton.whileTrue(new ZeroNavxMidMatch(drive));

    // OI.driverX.whileHeld(new MoveToPieceForwards(drive, peripherals, lights));

    // OI.operatorA.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 221), new RotateWrist(wrist, -123), new SetArmExtensionPosition(lights, armExtension, armRotation, 14)));
    // OI.operatorY.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 221), new RotateWrist(wrist, -127), new SetArmExtensionPosition(lights, armExtension, armRotation, 38)));
    
    // OI.operatorB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 269.5), new RotateWrist(wrist, -63)));

    // OI.operatorRB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 95), new RotateWrist(wrist, 132)));
    // OI.operatorLB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, 89), new RotateWrist(wrist, 132)));

    OI.driverRB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, 145), new RotateWrist(wrist, flipChecker, 121), new SetArmExtensionPosition(lights, armExtension, armRotation, 19)));

    // OI.operatorA.whileHeld(new RotateWrist(wrist, flipChecker, 50));
    // OI.operatorY.whileHeld(new RotateWrist(wrist, flipChecker, -50));

    // placement position mid
    OI.operatorA.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, Constants.PLACEMENT_PRESET_ARM_ROTATION_MID), new RotateWrist(wrist, flipChecker, Constants.PLACEMENT_PRESET_MID_WRIST_ROTATION), new SetArmExtensionPosition(lights, armExtension, armRotation, 12)));
    // placement position high
    OI.operatorY.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, Constants.PLACEMENT_PRESET_ARM_ROTATION_HIGH), new RotateWrist(wrist, flipChecker, Constants.PLACEMENT_PRESET_HIGH_WRIST_ROTATION), new SetArmExtensionPosition(lights, armExtension, armRotation, 37.5)));
    
    // intake position for upright cone
    OI.operatorX.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, Constants.UPRIGHT_CONE_PRESET_ARM_ROTATION), new RotateWrist(wrist, flipChecker, Constants.UPRIGHT_CONE_PRESET_WRIST_ROTATION)));
    // intake position for a tipped cone
    OI.operatorB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, Constants.TIPPED_CONE_PRESET_ARM_ROTATION), new RotateWrist(wrist, flipChecker, Constants.TIPPED_CONE_PRESET_WRIST_ROTATION)));

    OI.operatorRB.whileHeld(new ParallelCommandGroup(new SetArmRotationPosition(armRotation, flipChecker, Constants.CUBE_PRESET_ARM_ROTATION), new RotateWrist(wrist, flipChecker, Constants.CUBE_PRESET_WRIST_ROTATION)));
    // OI.operatorRB.whenRelease

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
