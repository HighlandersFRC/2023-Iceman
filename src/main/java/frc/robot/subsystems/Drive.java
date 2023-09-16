package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.defaults.DriveDefault;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class Drive extends SubsystemBase {
    private final OI OI = new OI();

    // interpolation range
    private double turnTimePercent = 0.3;

    private final double interpolationCorrection = 0;

    // cycle period of robot code
    private final double cyclePeriod = 1.0/50.0;

    private double[] velocityArray = new double[3];

    // creating all the falcons
    private final WPI_TalonFX leftForwardMotor = new WPI_TalonFX(3);
    private final WPI_TalonFX leftForwardAngleMotor = new WPI_TalonFX(4);
    private final WPI_TalonFX leftBackMotor = new WPI_TalonFX(5);
    private final WPI_TalonFX leftBackAngleMotor = new WPI_TalonFX(6);
    private final WPI_TalonFX rightForwardMotor = new WPI_TalonFX(1);
    private final WPI_TalonFX rightForwardAngleMotor = new WPI_TalonFX(2);
    private final WPI_TalonFX rightBackMotor = new WPI_TalonFX(7);
    private final WPI_TalonFX rightBackAngleMotor = new WPI_TalonFX(8);

    // creating peripherals object to access sensors
    private Peripherals peripherals;

    // xy position of module based on robot width and distance from edge of robot
    private final double moduleX = ((Constants.ROBOT_WIDTH)/2) - Constants.MODULE_OFFSET;
    private final double moduleY = ((Constants.ROBOT_LENGTH)/2) - Constants.MODULE_OFFSET;

    // creating all the external encoders
    private CANCoder backRightAbsoluteEncoder = new CANCoder(4);
    private CANCoder frontLeftAbsoluteEncoder = new CANCoder(2);
    private CANCoder frontRightAbsoluteEncoder = new CANCoder(1);
    private CANCoder backLeftAbsoluteEncoder = new CANCoder(3);

    // creating each swerve module with angle and drive motor, module number(relation to robot), and external encoder
    private final SwerveModule leftFront = new SwerveModule(2, leftForwardAngleMotor, leftForwardMotor, 0, frontLeftAbsoluteEncoder);
    private final SwerveModule leftBack = new SwerveModule(3, leftBackAngleMotor, leftBackMotor, 0, backLeftAbsoluteEncoder);
    private final SwerveModule rightFront = new SwerveModule(1, rightForwardAngleMotor, rightForwardMotor, 0, frontRightAbsoluteEncoder);
    private final SwerveModule rightBack = new SwerveModule(4, rightBackAngleMotor, rightBackMotor, 0, backRightAbsoluteEncoder);

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(moduleX, moduleY);
    Translation2d m_frontRightLocation = new Translation2d(moduleX, -moduleY);
    Translation2d m_backLeftLocation = new Translation2d(-moduleX, moduleY);
    Translation2d m_backRightLocation = new Translation2d(-moduleX, -moduleY);

    // values for odometry for autonomous pathing
    private double currentX = 0;
    private double currentY = 0;
    private double currentTheta = 0;

    private double estimatedX = 0.0;
    private double estimatedY = 0.0;
    private double estimatedTheta = 0.0;

    private double previousEstimateX = 0.0;
    private double previousEstimateY = 0.0;
    private double previousEstimateTheta = 0.0;

    private double averagedX = 0.0;
    private double averagedY = 0.0;
    private double averagedTheta = 0.0;

    private double initTime;
    private double currentTime;
    private double previousTime;
    private double timeDiff;

    private double previousX = 0;
    private double previousY = 0;
    private double previousTheta = 0;

    private double currentXVelocity = 0;
    private double currentYVelocity = 0;
    private double currentThetaVelocity = 0;

    private Boolean useCameraInOdometry = true;

    // location of the target in the center of the field
    private double targetCenterX = 8.2296;
    private double targetCenterY = 4.1148;

    // array for fused odometry
    private double[] currentFusedOdometry = new double[3];

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // odometry
    SwerveDrivePoseEstimator m_odometry; 
    Pose2d m_pose;

    double initAngle;
    double setAngle;
    double diffAngle;

    private double xP = 4;
    private double xI = 0;
    private double xD = 1.2;

    private double yP = 4;
    private double yI = 0;
    private double yD = 1.2;

    private double thetaP = 3.1;
    private double thetaI = 0;
    private double thetaD = 0.8;

    private PID xPID = new PID(xP, xI, xD);
    private PID yPID = new PID(yP, yI, yD);
    private PID thetaPID = new PID(thetaP, thetaI, thetaD);

    private int lookAheadDistance = 5;

    private double timeSinceLastCameraMeasurement = 0;

    private String fieldSide = "blue";

    public Drive(Peripherals peripherals) {
        this.peripherals = peripherals;
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(0, new Rotation2d(rightFront.getAbsolutePositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(0, new Rotation2d(leftFront.getAbsolutePositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(0, new Rotation2d(leftBack.getAbsolutePositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(0, new Rotation2d(rightBack.getAbsolutePositionRadians()));

        Pose2d m_pose = new Pose2d();

        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(peripherals.getNavxAngle()), swerveModulePositions, m_pose);
    }

    public void setFieldSide(String side){
        fieldSide = side;
    }

    public String getFieldSide(){
        return fieldSide;
    }

    public double getCurrentTime(){
        return currentTime;
    }

    // method to zeroNavx mid match and reset odometry with zeroed angle
    public void zeroNavxMidMatch() {
        peripherals.zeroNavx();
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(rightFront.getModuleDistance(), new Rotation2d(rightFront.getAbsolutePositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(leftFront.getModuleDistance(), new Rotation2d(leftFront.getAbsolutePositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(leftBack.getModuleDistance(), new Rotation2d(leftBack.getAbsolutePositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(rightBack.getModuleDistance(), new Rotation2d(rightBack.getAbsolutePositionRadians()));
        m_odometry.resetPosition(new Rotation2d(peripherals.getNavxAngle()), swerveModulePositions, new Pose2d(new Translation2d(getFusedOdometryX(), getFusedOdometryY()), new Rotation2d(peripherals.getNavxAngle())));
    }

    public void setNavxAfterAuto() {
        peripherals.setNavxAngle((peripherals.getNavxAngle() + 180)%360);
    }

    public void setNavxAngle(double angle){
        peripherals.setNavxAngle(angle);
    }

    public double getNavxAngle(){
        return peripherals.getNavxAngle();
    }

    public double getNavxPitch(){
        return peripherals.getNavxPitch();
    }

    public double getNavxRoll(){
        return peripherals.getNavxRoll();
    }

    // get each external encoder
    public double getLeftForwardEncoder() {
        return leftFront.getAbsolutePosition();
    }

    public double getLeftBackEncoder() {
        return leftBack.getAbsolutePosition();
    }

    public double getRightForwardEncoder() {
        return rightFront.getAbsolutePosition();
    }

    public double getRightBackEncoder() {
        return rightBack.getAbsolutePosition();
    }

    public void lockWheels() {
        leftBack.setAnglePID(Math.PI/4, 0);
        rightBack.setAnglePID(-Math.PI/4, 0);
        rightFront.setAnglePID(Math.PI/4, 0);
        leftFront.setAnglePID(-Math.PI/4, 0);
    }

    public void setWheelsHorizontal(){
        leftBack.setAnglePID(Math.PI / 2, 0);
        rightBack.setAnglePID(Math.PI / 2, 0);
        rightFront.setAnglePID(Math.PI / 2, 0);
        leftFront.setAnglePID(Math.PI / 2, 0);
    }

    public void setWheelsStraight(){
        leftBack.setAnglePID(0, 0);
        leftFront.setAnglePID(0, 0);
        rightFront.setAnglePID(0, 0);
        rightBack.setAnglePID(0, 0);
    }

    public double[] getWheelVelocities(){
        double[] vels = new double[4];
        vels[0] = rightFront.getModuleSpeed();
        vels[1] = leftFront.getModuleSpeed();
        vels[2] = leftBack.getModuleSpeed();
        vels[3] = rightBack.getModuleSpeed();
        return vels;
    }

    public double[] getWheelOutputs(){
        double[] vels = new double[4];
        vels[0] = rightFront.getDesiredVelocity();
        vels[1] = leftFront.getDesiredVelocity();
        vels[2] = leftBack.getDesiredVelocity();
        vels[3] = rightBack.getDesiredVelocity();
        return vels;
    }

    // get Joystick adjusted y-value
    public double getAdjustedY(double originalX, double originalY){
        double adjustedY = originalY * Math.sqrt((1-(Math.pow(originalX, 2))/2));
        return adjustedY;
    }

    // get Joystick adjusted x-value
    public double getAdjustedX(double originalX, double originalY){
        double adjustedX = originalX * Math.sqrt((1-(Math.pow(originalY, 2))/2));
        return adjustedX;
    }

    // method run on robot initialization
    public void init() {
        leftFront.init();
        leftBack.init();
        rightBack.init();
        rightFront.init();

        xPID.setMinOutput(-4.9);
        xPID.setMaxOutput(4.9);

        yPID.setMinOutput(-4.9);
        yPID.setMaxOutput(4.9);

        thetaPID.setMinOutput(-(Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));
        thetaPID.setMaxOutput((Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));

        setDefaultCommand(new DriveDefault(this));
    }

    public void teleopInit() {
        rightFront.teleopInit();
        leftFront.teleopInit();
        leftBack.teleopInit();
        rightBack.teleopInit();
    }

    // method run during start of autonomous
    public void autoInit(JSONArray pathPoints) {
        JSONArray firstPoint = pathPoints.getJSONArray(0);

        double firstPointX = firstPoint.getDouble(1);
        double firstPointY = firstPoint.getDouble(2);
        double firstPointAngle = firstPoint.getDouble(3);

        // changing odometry if on blue side, don't need to change y because it will be the same for autos on either side
        if(getFieldSide() == "blue") {
            firstPointX = Constants.FIELD_LENGTH - firstPointX;
            firstPointAngle = Math.PI - firstPointAngle;
        }

        // System.out.println("First point angle: " + firstPointAngle);
        
        peripherals.setNavxAngle(Math.toDegrees(firstPointAngle));
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(rightFront.getModuleDistance(), new Rotation2d(rightFront.getAbsolutePositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(leftFront.getModuleDistance(), new Rotation2d(leftFront.getAbsolutePositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(leftBack.getModuleDistance(), new Rotation2d(leftBack.getAbsolutePositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(rightBack.getModuleDistance(), new Rotation2d(rightBack.getAbsolutePositionRadians()));
        m_odometry.resetPosition(new Rotation2d(firstPointAngle), swerveModulePositions, new Pose2d(new Translation2d(firstPointX, firstPointY), new Rotation2d(firstPointAngle)));
        
        estimatedX = getOdometryX();
        estimatedY = getOdometryY();
        estimatedTheta = getOdometryAngle();

        previousEstimateX = estimatedX;
        previousEstimateY = estimatedY;
        previousEstimateTheta = estimatedTheta;

        currentX = getOdometryX();
        currentY = getOdometryY();
        currentTheta = getOdometryAngle();

        previousX = currentX;
        previousY = currentY;
        previousTheta = currentTheta;

        averagedX = (estimatedX + currentX)/2;
        averagedY = (estimatedY + currentY)/2;   
        averagedTheta = (estimatedTheta + currentTheta)/2;

        initTime = Timer.getFPGATimestamp();

        updateOdometryFusedArray();
    }

    

    public void useCameraInOdometry() {
        useCameraInOdometry = true;
    }

    // method to update odometry by fusing prediction, encoder tics, and camera values
    public void updateOdometryFusedArray() {
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        JSONArray cameraCoordinates = peripherals.getBackLimelightBasedPosition();

        double cameraBasedX = 0;
        double cameraBasedY = 0;

        Matrix<N3, N1> stdDeviation = new Matrix<>(Nat.N3(), Nat.N1());

        stdDeviation.set(0, 0, 0);
        stdDeviation.set(1, 0, 0);
        stdDeviation.set(2, 0, 0);

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        swerveModulePositions[0] = new SwerveModulePosition(rightFront.getModuleDistance(), new Rotation2d(rightFront.getAbsolutePositionRadians()));
        swerveModulePositions[1] = new SwerveModulePosition(leftFront.getModuleDistance(), new Rotation2d(leftFront.getAbsolutePositionRadians()));
        swerveModulePositions[2] = new SwerveModulePosition(leftBack.getModuleDistance(), new Rotation2d(leftBack.getAbsolutePositionRadians()));
        swerveModulePositions[3] = new SwerveModulePosition(rightBack.getModuleDistance(), new Rotation2d(rightBack.getAbsolutePositionRadians()));
        
        m_pose = m_odometry.update(new Rotation2d((navxOffset)), swerveModulePositions);

        currentX = getOdometryX();
        currentY = getOdometryY();
        currentTheta = navxOffset;

        if(useCameraInOdometry && cameraCoordinates.getDouble(0) != 0) {
            cameraBasedX = cameraCoordinates.getDouble(0);
            cameraBasedY = cameraCoordinates.getDouble(1);
            timeSinceLastCameraMeasurement = 0;
            Pose2d cameraBasedPosition = new Pose2d(new Translation2d(cameraBasedX, cameraBasedY), new Rotation2d(navxOffset));
            m_odometry.addVisionMeasurement(cameraBasedPosition, Timer.getFPGATimestamp() - (peripherals.getBackCameraLatency()/1000));
        }

        currentTime = Timer.getFPGATimestamp() - initTime;
        timeDiff = currentTime - previousTime;

        SmartDashboard.putNumber("X Diff", (currentX - previousX));
        SmartDashboard.putNumber("TimeDiff", timeDiff);

        averagedX = (currentX + averagedX)/2;
        averagedY = (currentY + averagedY)/2;
        averagedTheta = (currentTheta + averagedTheta)/2;

        previousX = averagedX;
        previousY = averagedY;
        previousTheta = averagedTheta;
        previousTime = currentTime;
        previousEstimateX = estimatedX;
        previousEstimateY = estimatedY;
        previousEstimateTheta = estimatedTheta;

        currentFusedOdometry[0] = averagedX;
        currentFusedOdometry[1] = averagedY;
        currentFusedOdometry[2] = currentTheta;

        // System.out.println("X: " + averagedX + " Y: " + averagedY);
    }

    public double getFrontRightModuleVelocity() {
        return rightFront.getModuleSpeed();
    }

    public double getFrontRightModuleDistance() {
        return rightFront.getModuleDistance();
    }

    public double getCurrentXVelocity() {
        return currentXVelocity;
    }

    public double getCurrentYVelocity() {
        return currentYVelocity;
    }
    public double getCurrentThetaVelocity() {
        return currentThetaVelocity;
    }

    // getFusedOdometryX
    public double getFusedOdometryX() {
        return currentFusedOdometry[0];
    }

    // getFusedOdometryY
    public double getFusedOdometryY() {
        return currentFusedOdometry[1];
    }

    // getFusedOdometryTheta
    public double getFusedOdometryTheta() {
        return currentFusedOdometry[2];
    }

    // getDistance to the target based on odometry
    public double getDistanceToTarget() {
        double xDist = targetCenterX - getFusedOdometryX();
        double yDist = targetCenterY - getFusedOdometryY();

        double distToTarget = Math.sqrt((Math.pow(xDist, 2) + Math.pow(yDist, 2)));

        return distToTarget;
    }

    // get odometry values
    public double getOdometryX() {
        // return currentFusedOdometry[0];
        return m_odometry.getEstimatedPosition().getX();
    }

    public double getOdometryY() {
        // return currentFusedOdometry[1];
        return m_odometry.getEstimatedPosition().getY();
    }

    public double getOdometryAngle() {
        // return currentFusedOdometry[2];
        return m_odometry.getEstimatedPosition().getRotation().getRadians();
    }

    public Pose2d getDriveOdometry() {
        return m_pose;
    }

    // method to optimize which way to turn for autonomous when spinning
    public double getShortestAngle(double point1Angle, double point2Angle) {
        double op1 = 0;
        double op2 = 0;
        if(point1Angle >= point2Angle) {
            op2 = ((Math.PI * 2) - (point1Angle - point2Angle));
            op1 = (point1Angle - point2Angle);
        }
        else {
            op1 = ((Math.PI * 2) - (point2Angle - point1Angle));
            op2 = (point2Angle - point1Angle);
        }

        if(op1 <= op2) {
            return -op1;
        }
        else {
            return op2;
        }
    }

    public void robotCentericDrive() {
        updateOdometryFusedArray();

        double turnLimit = 0.4;

        double speedLimit = 0.5;

        // this is correct, X is forward in field, so originalX should be the y on the joystick
        double originalX = speedLimit * (Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = speedLimit * (Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double turn = turnLimit * (-(Math.copySign(OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));
        // turn = 0;
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        leftFront.velocityDrive(controllerVector, turn, 0);
        rightFront.velocityDrive(controllerVector, turn, 0);
        leftBack.velocityDrive(controllerVector, turn, 0);
        rightBack.velocityDrive(controllerVector, turn, 0);
    }

    public void autoRobotCentricDrive(Vector velocityVector, double turnRadiansPerSec){
        updateOdometryFusedArray();
        leftFront.velocityDrive(velocityVector, turnRadiansPerSec, 0);
        rightFront.velocityDrive(velocityVector, turnRadiansPerSec, 0);
        leftBack.velocityDrive(velocityVector, turnRadiansPerSec, 0);
        rightBack.velocityDrive(velocityVector, turnRadiansPerSec, 0);
    }

    public void driveAutoAligned(double degreesFromPlacement) {
        updateOdometryFusedArray();

        double turnRadiansPerSec = degreesFromPlacement;

        double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        leftFront.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        rightFront.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        leftBack.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
        rightBack.velocityDrive(controllerVector, turnRadiansPerSec, navxOffset);
    }


    // method run in teleop that accepts controller values to move swerve drive
    public void teleopDrive() {
        updateOdometryFusedArray();
        double turnCoefficient = 0.5;
        double speedCoefficient = 1;

        if(OI.driverController.getLeftBumper()) {
            // activate speedy spin
            turnCoefficient = 1;
        }
        // else {
        //     turnCoefficient = 0.75;

        // }

        // this is correct, X is forward in field, so originalX should be the y on the joystick
        double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

        if(Math.abs(originalX) < 0.075) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.075) {
            originalY = 0;
        }

        double turn = turnCoefficient * ((Math.copySign(OI.getDriverRightX() * OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));
        // turn = 0;
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        // navxOffset = 0;
        double xPower = getAdjustedX(originalX, originalY) * speedCoefficient;
        double yPower = getAdjustedY(originalX, originalY) * speedCoefficient;

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        leftFront.velocityDrive(controllerVector, turn, navxOffset);
        rightFront.velocityDrive(controllerVector, turn, navxOffset);
        leftBack.velocityDrive(controllerVector, turn, navxOffset);
        rightBack.velocityDrive(controllerVector, turn, navxOffset);
    }

    // method run in autonomous that accepts a velocity vector of xy velocities, as well as how much to spin per second
    public void autoDrive(Vector velocityVector, double turnRadiansPerSec) {
        updateOdometryFusedArray();
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        leftFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightFront.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        leftBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
        rightBack.velocityDrive(velocityVector, turnRadiansPerSec, navxOffset);
    }

    // safely divide
    public double safeDivision(double numerator, double denominator) {
        if(Math.abs(denominator) < 0.00001) {
            return 0.0;
        }
        double dividend = numerator/denominator;
        return dividend;
    }

    public double samplePosition(RealMatrix coefficientsMatrix, double time) {
        ArrayList<Double> coefs = new ArrayList<Double>();
        for (double[] coef : coefficientsMatrix.getData()){
            coefs.add(coef[0]);
        }
        // System.out.println(coefs.toString());
        double sampled = 0.0;
        sampled += coefs.get(5) * Math.pow(time, 5);
        sampled += coefs.get(4) * Math.pow(time, 4);
        sampled += coefs.get(3) * Math.pow(time, 3);
        sampled += coefs.get(2) * Math.pow(time, 2);
        sampled += coefs.get(1) * Math.pow(time, 1);
        sampled += coefs.get(0);
        // System.out.println(sampled);
        // System.out.println("Time: " + time);
        return sampled;
    }

    // return the closest placement group 1, 4, or 7 (left to right)
    // for instance, on red side group 1 is the leftmost (in the driver's perspective) group, with tag ID 1
    // blue side group 7 is the rightmost group, with tag ID 8
    public int getClosestPlacementGroup(String fieldSide, double robotX, double robotY) {
        double distance = 999999;
        int number = 0;
        if (fieldSide == "red"){
            for (int i = 0; i < 3; i ++){
                double[] tagPosition = Constants.TAG_LOCATIONS[i];
                double tagDistance = Math.sqrt(Math.pow(tagPosition[0] - robotX, 2) + Math.pow(tagPosition[1] - robotY, 2));
                if (tagDistance < distance){
                    distance = tagDistance;
                    number = i * 3 + 1;
                }
            }
        } else if (fieldSide == "blue"){
            for (int i = 5; i < 8; i ++){
                double[] tagPosition = Constants.TAG_LOCATIONS[i];
                double tagDistance = Math.sqrt(Math.pow(tagPosition[0] - robotX, 2) + Math.pow(tagPosition[1] - robotY, 2));
                if (tagDistance < distance){
                    distance = tagDistance;
                    number = (i - 5) * 3 + 1;
                }
            }
        }
        return number;
    }

    // points are defined as (time, x, y, theta, vx, vy, vtheta, ax, ay, atheta)
    // generates a path to place a object on the fly, input is which part of grid to place on (0-8)
    // placement location 0-8, from left to right
    public JSONArray generatePlacementPathOnTheFly(int placementLocation, String fieldSide) {
        useCameraInOdometry = false;
        updateOdometryFusedArray();
        double[] firstPoint = new double[] {0, getFusedOdometryX(), getFusedOdometryY(), getFusedOdometryTheta(), 0, 0, 0, 0, 0, 0};
        // double[] firstPoint = new double[] {0, 14.3, 2.8, Math.toRadians(-180), 0, 0, 0, 0, 0, 0};
        double robotX = getFusedOdometryX();
        double robotY = getFusedOdometryY();
        double robotAngle = getFusedOdometryTheta();

        // System.out.println(robotAngle);

        // double placementAngle = Math.toRadians(-180);

        // if(placementAngle - robotAngle >= Math.PI) {
        //     placementAngle = placementAngle - (2 * Math.PI);
        // }
        // else if(placementAngle - robotAngle <= -Math.PI){
        //     placementAngle = placementAngle + (2 * Math.PI);
        // }

        double[] midPoint;
        double[] placementPoint;

        double pathAngle;
        if (robotAngle <= Math.toRadians(-90.0)) {
            pathAngle = Math.toRadians(-180.0);
        } else if (robotAngle >= Math.toRadians(90)) {
            pathAngle = Math.toRadians(180.0);
        } else {
            pathAngle = 0.0;
        }

        SmartDashboard.putNumber("NavxAngle", getFusedOdometryTheta());
        SmartDashboard.putNumber("PathAngle", pathAngle);

        // if (fieldSide == "red"){
        midPoint = new double[] {1.5, Constants.PLACEMENT_PATH_MIDPOINT_X_RED, Constants.PLACEMENT_PATH_MIDPOINT_Y_RED[placementLocation], pathAngle, (Constants.PLACEMENT_LOCATION_X_RED - getFusedOdometryX())/3, (Constants.PLACEMENT_LOCATIONS_Y_RED[placementLocation] - getFusedOdometryY())/3, 0, 0, 0, 0};
        placementPoint = new double[] {3, Constants.PLACEMENT_LOCATION_X_RED, Constants.PLACEMENT_LOCATIONS_Y_RED[placementLocation], pathAngle, 0, 0, 0, 0, 0, 0};
            // midPoint = new double[] {1.5, robotX + 1.0, robotY, 0, 0, 0, 0, 0, 0, 0};
            // placementPoint = new double[] {3.0, robotX + 1.0, robotY, 0, 0, 0, 0, 0, 0, 0};
        // } else {
        //     midPoint = new double[] {};
        //     placementPoint = new double[] {};
        // }
        // System.out.print("Placement: " + Arrays.toString(placementPoint));

        double[] currentPoint;
        double[] nextPoint;

        ArrayList<RealMatrix> xEquations = new ArrayList<RealMatrix>();
        ArrayList<RealMatrix> yEquations = new ArrayList<RealMatrix>();
        ArrayList<RealMatrix> thetaEquations = new ArrayList<RealMatrix>();

        ArrayList<double[]> sampledPoints = new ArrayList<double[]>();

        for(int i = 0; i < 2; i++) {
            if(i == 0) {
                currentPoint = firstPoint;
                nextPoint = midPoint;
            }
            else {
                currentPoint = midPoint;
                nextPoint = placementPoint;
            }
            double time = currentPoint[0];
            double currentX = currentPoint[1];
            double currentY = currentPoint[2];
            double currentTheta = currentPoint[3];
            double currentXVelocity = currentPoint[4];
            double currentYVelocity = currentPoint[5];
            double currentThetaVelocity = currentPoint[6];
            double currentXAccel = currentPoint[7];
            double currentYAccel = currentPoint[8];
            double currentThetaAccel = currentPoint[9];

            double nextTime = nextPoint[0];
            double nextX = nextPoint[1];
            double nextY = nextPoint[2];
            double nextTheta = nextPoint[3];
            double nextXVelocity = nextPoint[4];
            double nextYVelocity = nextPoint[5];
            double nextThetaVelocity = nextPoint[6];
            double nextXAccel = nextPoint[7];
            double nextYAccel = nextPoint[8];
            double nextThetaAccel = nextPoint[9];

            double[] firstPointEq;
            double[] secondPointEq;
            double[] firstVelEq;
            double[] secondVelEq;
            double[] firstAccelEq;
            double[] secondAccelEq;

            if(i == 0) {
                firstPointEq = new double[] {1, time, Math.pow(time, 2), Math.pow(time, 3), Math.pow(time, 4), Math.pow(time, 5)};
                secondPointEq = new double[] {1, nextTime, Math.pow(nextTime, 2), Math.pow(nextTime, 3), Math.pow(nextTime, 4), Math.pow(nextTime, 5),};
                firstVelEq = new double[] {0, 1, 2 * (time), 3 * (Math.pow(time, 2)), 4 * (Math.pow(time, 3)), 5 * (Math.pow(time, 4))};
                secondVelEq = new double[] {0, 1, 2 * (nextTime), 3 * (Math.pow(nextTime, 2)), 4 * (Math.pow(nextTime, 3)), 5 * (Math.pow(nextTime, 4))};
                firstAccelEq = new double[] {0, 0, 2, 6 * time, 12 * (Math.pow(time, 2)), 20 * ((Math.pow(time, 3)))};
                secondAccelEq = new double[] {0, 0, 2, 6 * nextTime, 12 * (Math.pow(nextTime, 2)), 20 * ((Math.pow(nextTime, 3)))};
            }
            else {
                firstPointEq = new double[] {1, time, Math.pow(time, 2), Math.pow(time, 3), Math.pow(time, 4), Math.pow(time, 5)};
                secondPointEq = new double[] {1, nextTime, Math.pow(nextTime, 2), Math.pow(nextTime, 3), Math.pow(nextTime, 4), Math.pow(nextTime, 5)};
                firstVelEq = new double[] {0, 1, 2 * (time), 3 * (Math.pow(time, 2)), 4 * (Math.pow(time, 3)), 5 * (Math.pow(time, 4))};
                secondVelEq = new double[] {0, 1, 2 * (nextTime), 3 * (Math.pow(nextTime, 2)), 4 * (Math.pow(nextTime, 3)), 5 * (Math.pow(nextTime, 4))};
                firstAccelEq = new double[] {0, 0, 2, 6 * time, 12 * (Math.pow(time, 2)), 20 * ((Math.pow(time, 3)))};
                secondAccelEq = new double[] {0, 0, 2, 6 * nextTime, 12 * (Math.pow(nextTime, 2)), 20 * ((Math.pow(nextTime, 3)))};
            }

            double[] xArray = new double[] {currentX, nextX, currentXVelocity, nextXVelocity, currentXAccel, nextXAccel};
            double[] yArray = new double[] {currentY, nextY, currentYVelocity, nextYVelocity, currentYAccel, nextYAccel};
            double[] thetaArray = new double[] {currentTheta, nextTheta, currentThetaVelocity, nextThetaVelocity, currentThetaAccel, nextThetaAccel};

            double[][] equationData = {firstPointEq, secondPointEq, firstVelEq, secondVelEq, firstAccelEq, secondAccelEq};

            RealMatrix systemEqMatrix = MatrixUtils.createRealMatrix(equationData);

            RealMatrix xMatrix = MatrixUtils.createColumnRealMatrix(xArray);
            RealMatrix yMatrix = MatrixUtils.createColumnRealMatrix(yArray);
            RealMatrix thetaMatrix = MatrixUtils.createColumnRealMatrix(thetaArray);

            RealMatrix systemEqMatrixInverse = MatrixUtils.inverse(systemEqMatrix);

            RealMatrix xCoefficients = systemEqMatrixInverse.multiply(xMatrix);
            RealMatrix yCoefficients = systemEqMatrixInverse.multiply(yMatrix);
            RealMatrix thetaCoefficients = systemEqMatrixInverse.multiply(thetaMatrix);

            xEquations.add(xCoefficients);
            yEquations.add(yCoefficients);
            thetaEquations.add(thetaCoefficients);
        }
        // sample points along the path
        double time = firstPoint[0];
        while (time <= placementPoint[0]){
            int equationIndex;
            if (time <= midPoint[0]){
                equationIndex = 0;
            } else {
                equationIndex = 1;
            }
            double x = samplePosition(xEquations.get(equationIndex), time);
            double y = samplePosition(yEquations.get(equationIndex), time);
            double theta = samplePosition(thetaEquations.get(equationIndex), time);
            sampledPoints.add(new double[] {time, x, y, theta});
            time += 0.02;
        }

        JSONArray path = new JSONArray(sampledPoints);
        // System.out.println(path.toString());
        return path;
    }

    // Autonomous algorithm
    public double[] pidController(double currentX, double currentY, double currentTheta,double time, JSONArray pathPoints) {
        // System.out.println(pathPoints.toString());
        if(time < pathPoints.getJSONArray(pathPoints.length() - 1).getDouble(0)) {
            JSONArray currentPoint = pathPoints.getJSONArray(0);
            JSONArray targetPoint = pathPoints.getJSONArray(0);
            for(int i = 0; i < pathPoints.length(); i ++) {
                if(i == pathPoints.length() - lookAheadDistance) {
                    currentPoint = pathPoints.getJSONArray(i + 1);
                    targetPoint = pathPoints.getJSONArray((i + (lookAheadDistance - 1)));
                    break;
                }

                currentPoint = pathPoints.getJSONArray(i + 1);
                JSONArray previousPoint = pathPoints.getJSONArray(i);
                
                double currentPointTime = currentPoint.getDouble(0);
                double previousPointTime = previousPoint.getDouble(0);

                if(time > previousPointTime && time < currentPointTime){
                    targetPoint = pathPoints.getJSONArray(i + (lookAheadDistance - 1));
                    break;
                }
            }
            
            double targetTime = targetPoint.getDouble(0);
            double targetX = targetPoint.getDouble(1);
            double targetY = targetPoint.getDouble(2);
            double targetTheta = targetPoint.getDouble(3);

            if(getFieldSide() == "blue") {
                targetX = Constants.FIELD_LENGTH - targetX;
                targetTheta = Math.PI - targetTheta;
            }

            if (targetTheta - currentTheta > Math.PI){
                targetTheta -= 2 * Math.PI;
            } else if (targetTheta - currentTheta < -Math.PI){
                targetTheta += 2 * Math.PI;
            }

            double currentPointTime = currentPoint.getDouble(0);
            double currentPointX = currentPoint.getDouble(1);
            double currentPointY = currentPoint.getDouble(2);
            double currentPointTheta = currentPoint.getDouble(3);

            if(getFieldSide() == "blue") {
                currentPointX = Constants.FIELD_LENGTH - currentPointX;
                currentPointTheta = Math.PI - currentPointTheta;
            }

            double feedForwardX = (targetX - currentPointX)/(targetTime - currentPointTime);
            double feedForwardY = (targetY - currentPointY)/(targetTime - currentPointTime);
            double feedForwardTheta = -(targetTheta - currentPointTheta)/(targetTime - currentPointTime);

            xPID.setSetPoint(targetX);
            yPID.setSetPoint(targetY);
            thetaPID.setSetPoint(targetTheta);

            xPID.updatePID(currentX);
            yPID.updatePID(currentY);
            thetaPID.updatePID(currentTheta);

            double xVelNoFF = xPID.getResult();
            double yVelNoFF = yPID.getResult();
            double thetaVelNoFF = -thetaPID.getResult();

            double xVel = feedForwardX + xVelNoFF;
            double yVel = feedForwardY + yVelNoFF;
            double thetaVel = feedForwardTheta + thetaVelNoFF;

            double[] velocityArray = new double[3];

            velocityArray[0] = xVel;
            velocityArray[1] = yVel;
            velocityArray[2] = thetaVel;

            // System.out.println("Target Point: " + targetPoint);

            return velocityArray;
        }
        else {
            double[] velocityArray = new double[3];

            velocityArray[0] = 0;
            velocityArray[1] = 0;
            velocityArray[2] = 0;

            return velocityArray;
        }
        
    }
 
    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}