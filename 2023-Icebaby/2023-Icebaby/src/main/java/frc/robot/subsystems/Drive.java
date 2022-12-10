package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
    private final double moduleXY = ((Constants.ROBOT_WIDTH)/2) - Constants.MODULE_OFFSET;

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
    Translation2d m_frontLeftLocation = new Translation2d(moduleXY, moduleXY);
    Translation2d m_frontRightLocation = new Translation2d(moduleXY, -moduleXY);
    Translation2d m_backLeftLocation = new Translation2d(-moduleXY, moduleXY);
    Translation2d m_backRightLocation = new Translation2d(-moduleXY, -moduleXY);

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
    SwerveDriveOdometry m_odometry; 
    Pose2d m_pose;

    double initAngle;
    double setAngle;
    double diffAngle;

    private double xP = 1.5;
    private double xI = 0;
    private double xD = 1;

    private double yP = 1.5;
    private double yI = 0;
    private double yD = 1;

    private double thetaP = 4.5;
    private double thetaI = 0;
    private double thetaD = 0.4;

    private PID xPID = new PID(xP, xI, xD);
    private PID yPID = new PID(yP, yI, yD);
    private PID thetaPID = new PID(thetaP, thetaI, thetaD);

    private int lookAheadDistance = 3;

    public Drive(Peripherals peripherals) {
        this.peripherals = peripherals;

        m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(Math.toRadians(peripherals.getNavxAngle())));
    }

    // method to zeroNavx mid match and reset odometry with zeroed angle
    public void zeroNavxMidMatch() {
        peripherals.zeroNavx();
        m_odometry.resetPosition(new Pose2d(new Translation2d(getFusedOdometryX(), getFusedOdometryY()), new Rotation2d(getFusedOdometryTheta())), new Rotation2d(getFusedOdometryTheta()));
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
        leftBack.setAnglePID(-Math.PI/4, 0);
        rightBack.setAnglePID(Math.PI/4, 0);
        rightFront.setAnglePID(-Math.PI/4, 0);
        leftFront.setAnglePID(Math.PI/4, 0);
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

    // method run during start of autonomous
    public void autoInit(JSONArray pathPoints) {
        JSONArray firstPoint = pathPoints.getJSONArray(0);

        double firstPointX = firstPoint.getDouble(1);
        double firstPointY = firstPoint.getDouble(2);
        double firstPointAngle = firstPoint.getDouble(3);

        System.out.println(firstPointAngle);
        
        peripherals.setNavxAngle(Math.toDegrees(firstPointAngle));
        m_odometry.resetPosition(new Pose2d(new Translation2d(firstPointX, firstPointY),  new Rotation2d(firstPointAngle)), new Rotation2d(firstPointAngle));
        
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
        // System.out.println("X: " + getFusedOdometryX() + " Y: " + getFusedOdometryY() + " THETA: "+ getFusedOdometryTheta());
    }

    // method to update odometry by fusing prediction, encoder tics, and camera values
    public void updateOdometryFusedArray() {
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        double cameraDistanceToTarget = peripherals.getLimeLightDistanceToTarget();

        // double[] cameraCoordinates = peripherals.calculateRobotPosFromCamera();

        // double cameraXVal = cameraCoordinates[0];
        // double cameraYVal = cameraCoordinates[1];
        
        m_pose = m_odometry.update(new Rotation2d((navxOffset)), leftFront.getState((navxOffset)), rightFront.getState((navxOffset)), leftBack.getState((navxOffset)), rightBack.getState((navxOffset)));

        currentX = getOdometryX();
        currentY = getOdometryY();
        currentTheta = navxOffset;

        currentTime = Timer.getFPGATimestamp() - initTime;
        timeDiff = currentTime - previousTime;

        // determine current velocities based on current position minus previous position divided by time difference
        currentXVelocity = (currentX - previousX)/timeDiff;
        currentYVelocity = (currentY - previousY)/timeDiff;
        currentThetaVelocity = (currentTheta - previousTheta)/timeDiff;

        // determine estimated position by integrating current velocity by time and adding previous estimated position
        estimatedX = previousX + (cyclePeriod * currentXVelocity);
        estimatedY = previousY + (cyclePeriod * currentYVelocity);
        estimatedTheta = previousTheta + (cyclePeriod * currentThetaVelocity);

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

        System.out.println("X: " + averagedX + " Y: " + averagedY + " Theta: " + currentTheta);

        // m_odometry.resetPosition(new Pose2d(new Translation2d(averagedX, averagedY),  new Rotation2d(navxOffset)), new Rotation2d(navxOffset));
    }

    public void setOdometryXYTheta(double x, double y) {
        m_odometry.resetPosition(new Pose2d(new Translation2d(x, y),  new Rotation2d(Math.PI/2)), new Rotation2d(Math.PI/2));
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
        return m_odometry.getPoseMeters().getX();
    }

    public double getOdometryY() {
        // return currentFusedOdometry[1];
        return m_odometry.getPoseMeters().getY();
    }

    public double getOdometryAngle() {
        // return currentFusedOdometry[2];
        return m_odometry.getPoseMeters().getRotation().getRadians();
    }

    // update swerve odometry based on encoder tics
    public void updateOdometry(double navxOffset) {
        m_pose = m_odometry.update(new Rotation2d(Math.toRadians(-navxOffset)), leftFront.getState(Math.toRadians(-navxOffset)), rightFront.getState(Math.toRadians(-navxOffset)), leftBack.getState(Math.toRadians(-navxOffset)), rightBack.getState(Math.toRadians(-navxOffset)));
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

        // String strOdomList = (odometryList[0] + "," + odometryList[1] + ","+ odometryList[2]); 
        // // System.out.println("STRING ODOM LIST:" + strOdomList);

        // MqttMessage message = new MqttMessage(strOdomList.getBytes());

        // publish.publish("/pathTool", message);

        // System.out.println("X: " + getFusedOdometryX() + " Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        leftFront.velocityDrive(controllerVector, turn, 0);
        rightFront.velocityDrive(controllerVector, turn, 0);
        leftBack.velocityDrive(controllerVector, turn, 0);
        rightBack.velocityDrive(controllerVector, turn, 0);

        // leftFront.testDrive();
        // rightFront.testDrive();
        // leftBack.testDrive();
        // rightBack.testDrive();

    }

    // NOT WORKING method to accepts controller input of which way to drive but stay aligned to the target
    public void driveAutoAligned(double cameraTurnAdjustment) {
        updateOdometryFusedArray();

        double turn = (-OI.getDriverRightX() * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));

        // double cameraTurnAdjustment = -peripherals.getLimeLightX();

        double turnRadiansPerSec = turn + cameraTurnAdjustment;

        double originalX = -OI.getDriverLeftY();
        double originalY = -OI.getDriverLeftX();

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

        double turnLimit = 1;

        if(OI.driverController.getLeftBumper()) {
            // activate speedy spin
            turnLimit = 1;
        }
        else {
            turnLimit = 0.75;

        }
        // System.out.println("FUSED X: " + getFusedOdometryX() + " FUSED Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        // this is correct, X is forward in field, so originalX should be the y on the joystick
        double originalX = -(Math.copySign(OI.getDriverLeftY() * OI.getDriverLeftY(), OI.getDriverLeftY()));
        double originalY = -(Math.copySign(OI.getDriverLeftX() * OI.getDriverLeftX(), OI.getDriverLeftX()));

        if(Math.abs(originalX) < 0.05) {
            originalX = 0;
        }
        if(Math.abs(originalY) < 0.05) {
            originalY = 0;
        }

        double turn = turnLimit * ((Math.copySign(OI.getDriverRightX() * OI.getDriverRightX(), OI.getDriverRightX())) * (Constants.TOP_SPEED)/(Constants.ROBOT_RADIUS));
        // turn = 0;
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());
        // navxOffset = 0;
        double xPower = getAdjustedX(originalX, originalY);
        double yPower = getAdjustedY(originalX, originalY);

        double xSpeed = xPower * Constants.TOP_SPEED;
        double ySpeed = yPower * Constants.TOP_SPEED;

        Vector controllerVector = new Vector(xSpeed, ySpeed);
        // Vector controllerVector = new Vector(3, 3);

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        // String strOdomList = (odometryList[0] + "," + odometryList[1] + ","+ odometryList[2]); 
        // // System.out.println("STRING ODOM LIST:" + strOdomList);

        // MqttMessage message = new MqttMessage(strOdomList.getBytes());

        // publish.publish("/pathTool", message);

        // System.out.println("X: " + getFusedOdometryX() + " Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        leftFront.velocityDrive(controllerVector, turn, navxOffset);
        rightFront.velocityDrive(controllerVector, turn, navxOffset);
        leftBack.velocityDrive(controllerVector, turn, navxOffset);
        rightBack.velocityDrive(controllerVector, turn, navxOffset);

        // leftFront.testDrive();
        // rightFront.testDrive();
        // leftBack.testDrive();
        // rightBack.testDrive();

    }

    // method run in autonomous that accepts a velocity vector of xy velocities, as well as how much to spin per second
    public void autoDrive(Vector velocityVector, double turnRadiansPerSec) {
        updateOdometryFusedArray();
        double navxOffset = Math.toRadians(peripherals.getNavxAngle());

        double[] odometryList = new double[3];

        odometryList[0] = getFusedOdometryX();
        odometryList[1] = getFusedOdometryY();
        odometryList[2] = getFusedOdometryTheta();

        // MqttMessage message = new MqttMessage(strOdomList.getBytes());

        // publish.publish("/pathTool", message);

        // System.out.println("X: " + getFusedOdometryX() + " Y: " + getFusedOdometryY() + " Theta: " + getFusedOdometryTheta());

        // m_pose = m_odometry.update(new Rotation2d(Math.toRadians(-navxOffset)), leftFront.getState(Math.toRadians(-navxOffset)), rightFront.getState(Math.toRadians(-navxOffset)), leftBack.getState(Math.toRadians(-navxOffset)), rightBack.getState(Math.toRadians(-navxOffset)));
        // m_pose = m_odometry.update(new Rotation2d(navxOffset), leftFront.getState(navxOffset), rightFront.getState(navxOffset), leftBack.getState(navxOffset), rightBack.getState(navxOffset));

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

    // Autonomous algorithm
    public double[] pidController(double currentX, double currentY, double currentTheta,double time, JSONArray pathPoints) {
        if(time < pathPoints.getJSONArray(pathPoints.length() - 1).getDouble(0)) {
            JSONArray currentPoint = pathPoints.getJSONArray(0);
            JSONArray targetPoint = pathPoints.getJSONArray(0);
            for(int i = 0; i < pathPoints.length() - lookAheadDistance; i ++) {
                if(i == pathPoints.length() - lookAheadDistance) {
                    currentPoint = pathPoints.getJSONArray(i);
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

            double currentPointTime = currentPoint.getDouble(0);
            double currentPointX = currentPoint.getDouble(1);
            double currentPointY = currentPoint.getDouble(2);
            double currentPointTheta = currentPoint.getDouble(3);

            double feedForwardX = (targetX - currentPointX)/(targetTime - currentPointTime);
            double feedForwardY = (targetY - currentPointY)/(targetTime - currentPointTime);
            double feedForwardTheta = -(targetTheta - currentTheta)/(targetTime - time);

            if(feedForwardTheta > 2) {
                feedForwardTheta = 2;
            }
            else if(feedForwardTheta < -2) {
                feedForwardTheta = -2;
            }

            if (targetTheta - currentTheta > Math.PI){
                targetTheta -= 2 * Math.PI;
            } else if (targetTheta - currentTheta < -Math.PI){
                targetTheta += 2 * Math.PI;
            }

            xPID.setSetPoint(targetX);
            yPID.setSetPoint(targetY);
            thetaPID.setSetPoint(targetTheta);

            xPID.updatePID(currentX);
            yPID.updatePID(currentY);
            thetaPID.updatePID(currentTheta);

            double xVelNoFF = xPID.getResult();
            double yVelNoFF = yPID.getResult();
            double thetaVelNoFF = -thetaPID.getResult();

            double xVel = feedForwardX + (xVelNoFF);
            double yVel = feedForwardY + (yVelNoFF);
            double thetaVel = thetaVelNoFF;// + ((feedForwardTheta - thetaVelNoFF)/2);

            SmartDashboard.putNumber("XFF", feedForwardX);
            SmartDashboard.putNumber("YFF", feedForwardY);
            SmartDashboard.putNumber("THETAFF", feedForwardTheta);
            SmartDashboard.putNumber("TIME DIFF", (targetTime - time));

            double[] velocityArray = new double[3];

            velocityArray[0] = xVel;
            velocityArray[1] = yVel;
            velocityArray[2] = thetaVel;

            System.out.println("Target Point: " + targetPoint + " CURRENT TIME: " + time + " xVel: " + velocityArray[0] + " yVel: " + velocityArray[1] + " thetaVel: " + velocityArray[2]);

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