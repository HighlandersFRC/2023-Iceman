// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Peripherals;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // public static final double FIELD_WIDTH = 

    public static final double ROBOT_RADIUS = inchesToMeters(16.75); //m

    public static final double WHEEL_DIAMETER = inchesToMeters(4); //m

    public static final double WHEEL_CIRCUMFRENCE = Math.PI * WHEEL_DIAMETER; // m

    // number found through experimentation at top speed, official speed is around 16.3 f/sec
    public static final double TOP_SPEED = feetToMeters(20); // m/sec

    public static final double GEAR_RATIO = 6.75;

    public static final double STEER_GEAR_RATIO = 21.43;

    public static final double FALCON_TICS_PER_ROTATION = 2048;

    public static final double NEO_TICS_PER_ROTATION = 42;

    public static final double ROBOT_WIDTH = inchesToMeters(29); //m

    public static final double MODULE_OFFSET = inchesToMeters(2.5);

    public static final String CAMERA_NAME = "Limelight";

    public static final double CAMERA_CENTER_Z_OFFSET = inchesToMeters(-19);
    public static final double CAMERA_CENTER_X_OFFSET = 0;
    public static final double CAMERA_CENTER_Y_OFFSET = 0;
    public static final double CAMERA_ANGLE = Math.toRadians(37);
    public static final double CAMERA_PITCH = 0;
    public static final double CAMERA_YAW = 0;

    // location of tag on field, x
    public static final double TAG_ZERO_X = 8.236;
    // location of tag on field, y
    public static final double TAG_ZERO_Y = 3.47;
    // height of tag on field
    public static final double TAG_ZERO_Z = inchesToMeters(-58.5);
    // which way tag faces on the field
    public static final double TAG_ZERO_ANGLE_TO_FIELD = Math.toRadians(-90);
    // the way that the tag is angle from level on the field, in our case, it is not angle upward
    public static final double TAG_ZERO_PITCH_TO_FIELD = 0;
    // the way that the tag is turned in the field, in our case the tag is flat against the hub
    public static final double TAG_ZERO_YAW_TO_FIELD = 0;

    // see above for each definition for tag
    public static final double TAG_ONE_X = 7.6;
    public static final double TAG_ONE_Y = 4.11;
    public static final double TAG_ONE_Z = inchesToMeters(-58.5);
    public static final double TAG_ONE_ANGLE_TO_FIELD = Math.toRadians(180);
    public static final double TAG_ONE_PITCH_TO_FIELD = 0;
    public static final double TAG_ONE_YAW_TO_FIELD = 0;

    public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(0.4064, 0.0, 0.3302),
                        new Rotation3d(0, 0,0));

    public static RealMatrix getXRotationMatrix(double radians) {
        double[][] XMatrixData = {{1, 0, 0},
                                  {0, Math.cos(radians), -Math.sin(radians)},
                                  {0, Math.sin(radians), Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(XMatrixData);
    }

    public static RealMatrix getYRotationMatrix(double radians) {
        double[][] yMatrixData = {{Math.cos(radians), 0, Math.sin(radians)},
                                  {0, 1, 0},
                                  {-Math.sin(radians), 0, Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(yMatrixData);
    }

    public static RealMatrix getZRotationMatrix(double radians) {
        double[][] ZMatrixData = {{Math.cos(radians), -Math.sin(radians), 0},
                                  {Math.sin(radians), Math.cos(radians), 0},
                                  {0, 0, 1}};
        return MatrixUtils.createRealMatrix(ZMatrixData);
    }

    public static RealMatrix getRotationMatrix(double xRot, double yRot, double zRot) {
        RealMatrix xRotation = getXRotationMatrix(xRot);//should never exist unles tag is rotated on surface its mounted to
        RealMatrix yRotation = getYRotationMatrix(yRot);//when surface tag is mounted to is not perpendigular to ground
        RealMatrix zRotation = getZRotationMatrix(zRot);//when the tag is not directly infront of the robot, will chage most often

        RealMatrix xyzRotation = xRotation.multiply(yRotation).multiply(zRotation);        
        return xyzRotation;
    }
    /*
    public static RealMatrix getCameraToTagMatrix(double xRot, double yRot, double zRot, double cameraXinTagFrame, double cameraYinTagFrame, double cameraZinTagFrame) {
        RealMatrix cameraToTagRotationMatrix = getCameraToTagRotationMatrix(xRot, yRot, zRot);
        // RealMatrix inverseRotation = cameraToTagRotationMatrix.transpose();
        // inverseRotation = inverseRotation.scalarMultiply(-1);

        // double[] tMatrix = {cameraXinTagFrame, cameraYinTagFrame, cameraZinTagFrame};
        // RealMatrix translationMatrix = MatrixUtils.createColumnRealMatrix(tMatrix);

        // RealMatrix inversePerspective = inverseRotation.multiply(translationMatrix);
        
        // double[][] cameraToTag = { {cameraToTagRotationMatrix.getEntry(0, 0), cameraToTagRotationMatrix.getEntry(0, 1), cameraToTagRotationMatrix.getEntry(0, 2), inversePerspective.getRow(0)[0]}, {cameraToTagRotationMatrix.getEntry(1, 0), cameraToTagRotationMatrix.getEntry(1, 1), cameraToTagRotationMatrix.getEntry(1, 2), inversePerspective.getRow(1)[0]}, {cameraToTagRotationMatrix.getEntry(2, 0), cameraToTagRotationMatrix.getEntry(2, 1), cameraToTagRotationMatrix.getEntry(2, 2), inversePerspective.getRow(2)[0]}, {0, 0, 0, 1}};
        
        double[][] cameraToTag = { {cameraToTagRotationMatrix.getEntry(0, 0), cameraToTagRotationMatrix.getEntry(1, 0), cameraToTagRotationMatrix.getEntry(2, 0), -cameraXinTagFrame}, {cameraToTagRotationMatrix.getEntry(0, 1), cameraToTagRotationMatrix.getEntry(1, 1), cameraToTagRotationMatrix.getEntry(2, 1), -cameraYinTagFrame}, {cameraToTagRotationMatrix.getEntry(0, 2), cameraToTagRotationMatrix.getEntry(1, 2), cameraToTagRotationMatrix.getEntry(2, 2), -cameraZinTagFrame}, {0, 0, 0, 1}};

        return MatrixUtils.createRealMatrix(cameraToTag);
    }

    public static RealMatrix getCameraToRobotMatrix() {
        RealMatrix cameraRotationMatrix = getCameraRotationMatrix();

        double[][] cameraToRobot = { {cameraRotationMatrix.getEntry(0, 0), cameraRotationMatrix.getEntry(0, 1), cameraRotationMatrix.getEntry(0, 2), CAMERA_CENTER_X_OFFSET}, {cameraRotationMatrix.getEntry(1, 0), cameraRotationMatrix.getEntry(1, 1), cameraRotationMatrix.getEntry(1, 2), CAMERA_CENTER_Y_OFFSET}, {cameraRotationMatrix.getEntry(2, 0), cameraRotationMatrix.getEntry(2, 1), cameraRotationMatrix.getEntry(2, 2), CAMERA_CENTER_Z_OFFSET}, {0, 0, 0, 1}};

        return MatrixUtils.createRealMatrix(cameraToRobot);
    }

    public static RealMatrix getTagZeroToFieldMatrix() {
        RealMatrix TagZeroRotationMatrix = getTagZeroRotationMatrix();

        double[][] TagZeroToField = { {TagZeroRotationMatrix.getEntry(0, 0), TagZeroRotationMatrix.getEntry(0, 1), TagZeroRotationMatrix.getEntry(0, 2), TAG_ZERO_X}, {TagZeroRotationMatrix.getEntry(1, 0), TagZeroRotationMatrix.getEntry(1, 1), TagZeroRotationMatrix.getEntry(1, 2), TAG_ZERO_Y}, {TagZeroRotationMatrix.getEntry(2, 0), TagZeroRotationMatrix.getEntry(2, 1), TagZeroRotationMatrix.getEntry(2, 2), TAG_ZERO_Z}, {0, 0, 0, 1}};

        return MatrixUtils.createRealMatrix(TagZeroToField);
    }

    public static RealMatrix getTagOneToFieldMatrix() {
        RealMatrix TagOneRotationMatrix = getTagOneRotationMatrix();

        double[][] TagOneToField = { {TagOneRotationMatrix.getEntry(0, 0), TagOneRotationMatrix.getEntry(0, 1), TagOneRotationMatrix.getEntry(0, 2), TAG_ONE_X}, {TagOneRotationMatrix.getEntry(1, 0), TagOneRotationMatrix.getEntry(1, 1), TagOneRotationMatrix.getEntry(1, 2), TAG_ONE_Y}, {TagOneRotationMatrix.getEntry(2, 0), TagOneRotationMatrix.getEntry(2, 1), TagOneRotationMatrix.getEntry(2, 2), TAG_ONE_Z}, {0, 0, 0, 1}};

        return MatrixUtils.createRealMatrix(TagOneToField);
    }
*/
    // public static RealMatrix calculateCameraBasedPosition() {
    //     double tagID = Peripherals.getTargetID();
    //     if(tagID != -1) {
    //         double targetPitch = Peripherals.cameraPitchToTarget();
    //         double targetYaw = Peripherals.cameraYawToTarget();
    //         Transform3d camera3d = Peripherals.cameraToTarget();
    //         double tagX = 0;
    //         double tagY = 0;
    //         double tagZ = 0;
    //         double tagAngleToField = 0;

    //         if(tagID == 1) {
    //             tagX = TAG_ONE_X;
    //             tagY = TAG_ONE_Y;
    //             tagZ = TAG_ONE_Z;
    //             tagAngleToField = TAG_ONE_ANGLE_TO_FIELD;
    //         }

    //         System.out.println(camera3d.getX());
    //         System.out.println(camera3d.getY());

    //         double[] tagInCameraInformation = {camera3d.getX(), camera3d.getY(), camera3d.getZ()};

    //         RealMatrix tagInCameraMatrix = MatrixUtils.createColumnRealMatrix(tagInCameraInformation);

    //         double currentNavxAngle = Peripherals.getNavxAngle();

    //         double angleToTargetReferenceFrame = tagAngleToField - currentNavxAngle;

    //         RealMatrix cameraInTagRefRotation = getRotationMatrix(0, 0, angleToTargetReferenceFrame);

    //         RealMatrix cameraInTagReferenceMatrix = cameraInTagRefRotation.multiply(tagInCameraMatrix);

    //         // double[] cameraInTagInformation = {camera3d.getX(), camera3d.getY(), camera3d.getZ()};

    //         // RealMatrix cameraInTagMatrix = MatrixUtils.createColumnRealMatrix(cameraInTagInformation);
            
    //         // double angleToTarget = Math.atan2(camera3d.getY(), camera3d.getX()) - 180;
    //         // RealMatrix cameraToTagRotationMatrix = getRotationMatrix(0, 0, angleToTarget);//

    //         // RealMatrix cameraToTagTransformation = cameraToTagRotationMatrix.multiply(cameraInTagMatrix);
            
    //         // return cameraToTagTransformation;
           
    //     }
    //     RealMatrix noTarget = MatrixUtils.createRealIdentityMatrix(4);
    //     return noTarget;
        
    // }
/*
    public static RealMatrix testTagOneLocation() {
        double[] testLocation = {0, 0, 0, 1};

        RealMatrix testLocationMatrix = MatrixUtils.createColumnRealMatrix(testLocation);
        RealMatrix tagOneMatrix = getTagOneToFieldMatrix();

        RealMatrix testedTagLocation = tagOneMatrix.multiply(testLocationMatrix);

        return testedTagLocation;
    }

    public static RealMatrix testTagZeroLocation() {
        double[] testLocation = {0, 0, 0, 1};

        RealMatrix testLocationMatrix = MatrixUtils.createColumnRealMatrix(testLocation);
        RealMatrix tagZeroMatrix = getTagZeroToFieldMatrix();

        RealMatrix testedTagLocation = tagZeroMatrix.multiply(testLocationMatrix);

        return testedTagLocation;
    }

    public static RealMatrix testTagToRobot() {
        double[] robotCentricLocation = {0, 0, 0, 1};
        RealMatrix robotMatrix = MatrixUtils.createColumnRealMatrix(robotCentricLocation);

        RealMatrix tagToCameraMatrix = getCameraToTagMatrix(0, 0, Math.toRadians(16.7), 2.489, inchesToMeters(30), 1.5);
        RealMatrix cameraToRobotMatrix = getCameraToRobotMatrix();

        RealMatrix tagToRobotMatrix = tagToCameraMatrix.multiply(cameraToRobotMatrix);
        tagToRobotMatrix = tagToRobotMatrix.multiply(robotMatrix);
        
        return tagToRobotMatrix;
    }

    // public static RealMatrix testDistanceToTagOne() {
    //     double[] currentPosition = {7.51, 1.83, 0, 1};

    //     RealMatrix currentPositionMatrix = MatrixUtils.createColumnRealMatrix(currentPosition);
    //     RealMatrix tagOneToField = getTagOneToFieldMatrix();

    //     RealMatrix distanceToTarget = tagOneToField.multiply(currentPositionMatrix);

    //     return distanceToTarget;
    // }

    // public static RealMatrix testDistanceToTagZero() {
    //     double[] currentPosition = {7.51, 1.83, 0, 1};

    //     RealMatrix currentPositionMatrix = MatrixUtils.createColumnRealMatrix(currentPosition);
    //     RealMatrix tagZeroToField = getTagZeroToFieldMatrix();

    //     RealMatrix distanceToTarget = tagZeroToField.multiply(currentPositionMatrix);

    //     return distanceToTarget;
    // }
*/
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double feetToMeters(double feet) {
        double inches = feet * 12;
        return inchesToMeters(inches);
    }

    public static int shooterRPMToUnitsPer100MS(double rpm) {
        return (int) (Math.round((rpm / 600.0) * FALCON_TICS_PER_ROTATION));
    }

    public static double unitsPer100MsToRPM(double units) {
        return (units * 600) / (Constants.FALCON_TICS_PER_ROTATION);
    }

}
