// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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

    public static final String CAMERA_NAME = "LimeLight";

    public static final double CAMERA_CENTER_Z_OFFSET = inchesToMeters(19);
    public static final double CAMERA_CENTER_X_OFFSET = 0;
    public static final double CAMERA_CENTER_Y_OFFSET = 0;
    public static final double CAMERA_ANGLE = Math.toRadians(45);
    public static final double CAMERA_PITCH = 0;
    public static final double CAMERA_YAW = 0;

    // location of tag on field
    public static final double TAG_ZERO_X = 8.236;
    public static final double TAG_ZERO_Y = 3.248;
    // height of tag on field
    public static final double TAG_ZERO_Z = inchesToMeters(58.5);
    // which way tag faces on the field
    public static final double TAG_ZERO_ANGLE_TO_FIELD = Math.toRadians(180);
    // the way that the tag is angle from level on the field, in our case, it is not angle upward
    public static final double TAG_ZERO_PITCH_TO_FIELD = 0;
    // the way that the tag is turned in the field, in our case the tag is flat against the hub
    public static final double TAG_ZERO_YAW_TO_FIELD = 0;

    // see above for each definition for tag
    public static final double TAG_ONE_X = 7.413;
    public static final double TAG_ONE_Y = 4.104;
    public static final double TAG_ONE_Z = inchesToMeters(58.5);
    public static final double TAG_ONE_ANGLE_TO_FIELD = Math.toRadians(180);
    public static final double TAG_ONE_PITCH_TO_FIELD = 0;
    public static final double TAG_ONE_YAW_TO_FIELD = 0;

    public static RealMatrix getXRotationMatrix(double radians) {
        double[][] XMatrixData = {{1, 0, 0}, {0, Math.cos(radians), -Math.sin(radians)}, {0, Math.sin(radians), Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(XMatrixData);
    }

    public static RealMatrix getYRotationMatrix(double radians) {
        double[][] yMatrixData = {{Math.cos(radians), 0, Math.sin(radians)}, {0, 1, 0}, {-Math.sin(radians), 0, Math.cos(radians)}};
        return MatrixUtils.createRealMatrix(yMatrixData);
    }

    public static RealMatrix getZRotationMatrix(double radians) {
        double[][] ZMatrixData = {{Math.cos(radians), -Math.sin(radians), 0}, {Math.sin(radians), Math.cos(radians), 0}, {0, 0, 1}};
        return MatrixUtils.createRealMatrix(ZMatrixData);
    }

    public static RealMatrix getCameraRotationMatrix() {
        RealMatrix xRotation = getXRotationMatrix(CAMERA_PITCH);
        RealMatrix yRotation = getYRotationMatrix(CAMERA_YAW);
        RealMatrix zRotation = getZRotationMatrix(CAMERA_ANGLE);

        RealMatrix xyRotation = xRotation.multiply(yRotation);
        RealMatrix xyzRotation = xyRotation.multiply(zRotation);

        return xyzRotation;
    }

    public static RealMatrix getTagZeroRotationMatrix() {
        RealMatrix xRotation = getXRotationMatrix(TAG_ZERO_PITCH_TO_FIELD);
        RealMatrix yRotation = getYRotationMatrix(TAG_ZERO_YAW_TO_FIELD);
        RealMatrix zRotation = getZRotationMatrix(TAG_ZERO_ANGLE_TO_FIELD);

        RealMatrix xyRotation = xRotation.multiply(yRotation);
        RealMatrix xyzRotation = xyRotation.multiply(zRotation);

        return xyzRotation;
    }

    public static RealMatrix getTagOneRotationMatrix() {
        RealMatrix xRotation = getXRotationMatrix(TAG_ONE_YAW_TO_FIELD);
        RealMatrix yRotation = getYRotationMatrix(TAG_ONE_PITCH_TO_FIELD);
        RealMatrix zRotation = getZRotationMatrix(TAG_ONE_ANGLE_TO_FIELD);

        RealMatrix xyRotation = xRotation.multiply(yRotation);
        RealMatrix xyzRotation = xyRotation.multiply(zRotation);

        return xyzRotation;
    }

    public RealMatrix getCameraToRobotMatrix() {
        RealMatrix cameraRotationMatrix = getCameraRotationMatrix();

        double[][] cameraToRobot = { {cameraRotationMatrix.getEntry(0, 0), cameraRotationMatrix.getEntry(0, 1), cameraRotationMatrix.getEntry(0, 2), CAMERA_CENTER_X_OFFSET}, {cameraRotationMatrix.getEntry(1, 0), cameraRotationMatrix.getEntry(1, 1), cameraRotationMatrix.getEntry(1, 2), CAMERA_CENTER_X_OFFSET}, {cameraRotationMatrix.getEntry(2, 0), cameraRotationMatrix.getEntry(2, 1), cameraRotationMatrix.getEntry(2, 2), CAMERA_CENTER_Z_OFFSET}, {0, 0, 0, 1}};

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

    public static RealMatrix testDistanceToTagOne() {
        double[] currentPosition = {7.51, 1.83, 0, 1};

        RealMatrix currentPositionMatrix = MatrixUtils.createColumnRealMatrix(currentPosition);
        RealMatrix tagOneToField = getTagOneToFieldMatrix();

        RealMatrix distanceToTarget = tagOneToField.multiply(currentPositionMatrix);

        return distanceToTarget;
    }

    public static RealMatrix testDistanceToTagZero() {
        double[] currentPosition = {5.85, 1.63, 0, 1};

        RealMatrix currentPositionMatrix = MatrixUtils.createColumnRealMatrix(currentPosition);
        RealMatrix tagZeroToField = getTagZeroToFieldMatrix();

        RealMatrix distanceToTarget = tagZeroToField.multiply(currentPositionMatrix);

        return distanceToTarget;
    }

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
