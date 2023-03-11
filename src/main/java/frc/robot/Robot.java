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
import frc.robot.commands.DriveAutoAligned;
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
import frc.robot.commands.autos.OnePieceAuto;
import frc.robot.commands.autos.OnePieceAutoNoDock;
import frc.robot.commands.autos.TwoPieceAuto;
import frc.robot.commands.autos.TwoPieceAutoNoDock;
import frc.robot.commands.autos.TwoPieceBumpAuto;
import frc.robot.commands.autos.TwoPieceBumpAutoNoDock;
import frc.robot.commands.presets.CubePreset;
import frc.robot.commands.presets.HighPlacementPreset;
import frc.robot.commands.presets.MidPlacementPreset;
import frc.robot.commands.presets.RampIntakePreset;
import frc.robot.commands.presets.TippedConePreset;
import frc.robot.commands.presets.UprightConePreset;

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

  private boolean dpadClicked = false;

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

    // System.out.println(Constants.wristOffsetMidMatch);

    if(OI.operatorController.getPOV() == 0) {
      if(!dpadClicked) {
        Constants.increaseWristOffset();
      }
      dpadClicked = true;
    }
    else if(OI.operatorController.getPOV() == 180) {
      if(!dpadClicked) {
        Constants.decreaseWristOffset();
      }
      dpadClicked = true;
    }
    else {
      dpadClicked = false;
    }

    armRotation.postRotationValues();
    // System.out.println("ARM: " + armRotation.getRotationPosition());
    // System.out.println("WRIST: " + wrist.getWristRotationPosition());
    // System.out.println("EXTENSION: " + armExtension.getExtensionPosition());
    // lights.periodic();
    // System.out.println("Extension: " + armExtension.getExtensionPosition());

    //DO NOT COMMENT OUT!!!
    wrist.periodic();
    //DO NOT COMMENT OUT!!!
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if(OI.isBumpSideAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2PieceBumpPart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }
    else if(OI.isClearSideAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/2PiecePart1.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch(Exception e) {
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    } else if (OI.isCenterAuto()) {
      try {
        pathingFile = new File("/home/lvuser/deploy/1PieceDock.json");
        FileReader scanner = new FileReader(pathingFile);
        pathRead = new JSONObject(new JSONTokener(scanner));
        pathJSON = (JSONArray) pathRead.get("sampled_points");
      } catch (Exception e){
        System.out.println("ERROR WITH PATH FILE " + e);
      }
    }

    if (OI.isRedSide()) {
      drive.setFieldSide("red");
      lights.setFieldSide("red");
    } else if (OI.isBlueSide()) {
      drive.setFieldSide("blue");
      lights.setFieldSide("blue");
    }
    
    drive.autoInit(pathJSON);
    wrist.autoInit();

    if (OI.isBumpSideAuto()) {
      if (OI.isDocking()) {
        TwoPieceBumpAuto auto = new TwoPieceBumpAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      } else {
        TwoPieceBumpAutoNoDock auto = new TwoPieceBumpAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      }
    } else if (OI.isClearSideAuto()) {
      if (OI.isDocking()) {
        TwoPieceAuto auto = new TwoPieceAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      } else {
        TwoPieceAutoNoDock auto = new TwoPieceAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      }
    } else if (OI.isCenterAuto()) {
      if (OI.isDocking()){
        OnePieceAuto auto = new OnePieceAuto(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      } else {
        OnePieceAutoNoDock auto = new OnePieceAutoNoDock(drive, armExtension, armRotation, wrist, flipChecker, peripherals, lights, intake);
        auto.schedule();
      }
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drive.teleopInit(); 
    armExtension.teleopInit();
    wrist.teleopInit();

    OI.driverViewButton.whileTrue(new ZeroNavxMidMatch(drive));
    OI.driverX.whileHeld(new DriveAutoAligned(drive, peripherals, lights));

    // ramp intake position
    OI.operatorMenuButton.whileHeld(new RampIntakePreset(armExtension, armRotation, flipChecker, wrist, lights));

    // placement position mid
    OI.operatorA.whileHeld(new MidPlacementPreset(armExtension, armRotation, flipChecker, wrist, lights, peripherals));

    // placement position high
    OI.operatorY.whileHeld(new HighPlacementPreset(armExtension, armRotation, flipChecker, wrist, lights, peripherals));
    
    // intake position for upright cone
    OI.operatorX.whileHeld(new UprightConePreset(armExtension, armRotation, flipChecker, wrist, lights));
   
    // intake position for a tipped cone
    OI.operatorB.whileHeld(new TippedConePreset(armExtension, armRotation, flipChecker, wrist, lights));

    // intake position for cube
    OI.operatorRB.whileHeld(new CubePreset(armExtension, armRotation, flipChecker, wrist, lights));

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

    lights.periodic();

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
