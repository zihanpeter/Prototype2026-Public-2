package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Subsystem for the Mecanum Drive train using GoBilda Pinpoint Odometry for localization.
 * This class handles motor control, odometry updates, and movement logic.
 */
@Config
public class MecanumDrivePinpoint extends SubsystemBase {
    // Hardware devices
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private final GoBildaPinpointDriver pinpoint;
    
    // Offset for heading to allow resetting heading without resetting full odometry
    private double yawOffset;

    // Flag to indicate if gamepad control is active (used for braking logic)
    public boolean isGamepadOn;

    // Last recorded pose
    Pose2D lastPose;

    /**
     * Constructor for MecanumDrivePinpoint.
     * Initializes motors and the Pinpoint driver.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public MecanumDrivePinpoint(final HardwareMap hardwareMap) {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        // Initialize Pinpoint Driver (device name "od")
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        isGamepadOn = false;

        // Set Zero Power Behavior to BRAKE for all motors to resist movement when 0 power is applied
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pinpoint Configuration
        // Set offsets for the odometry pods relative to the robot center.
        // Constants are stored in inches in DriveConstants, but Pinpoint expects millimeters.
        // We pass values in inches and specify DistanceUnit.INCH.
        pinpoint.setOffsets(DriveConstants.xPoseDW, DriveConstants.yPoseDW, DistanceUnit.INCH);
        
        // Set the encoder resolution for the pods (GoBilda 4-Bar Pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        
        // Set encoder directions based on installation
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED, 
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        
        // Reset Position and IMU on initialization
        pinpoint.resetPosAndIMU();

        // Motor Directions
        // Updated based on user feedback: Forward/Strafe were reversed, Turn was correct.
        // Inverting all motors corrects Forward/Strafe while maintaining relative Turn direction.
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
    }

    /**
     * Stops the robot by setting all motor powers to 0.
     */
    public void stop() {
        moveRobot(0, 0, 0);
    }

    /**
     * Resets the robot's heading to a specified value.
     * This updates the yawOffset rather than resetting the Pinpoint hardware to preserve X/Y coordinates.
     *
     * @param heading The new heading in Radians.
     */
    public void reset(double heading) {
        // Update Pinpoint data before reading
        pinpoint.update();
        // Calculate offset so that (current_hardware_heading - offset) = new_desired_heading
        // offset = current_hardware_heading - new_desired_heading
        // Actually, logic in moveRobotFieldRelative is: botHeading = pinpoint.getHeading() - yawOffset;
        // So if we want botHeading = heading:
        // heading = pinpoint.getHeading() - yawOffset
        // yawOffset = pinpoint.getHeading() - heading
        
        // However, standard "reset" usually means "set current heading to 0".
        // If parameter 'heading' is the desired new heading:
        yawOffset = pinpoint.getHeading(DriveConstants.angleUnit) - heading;
    }

    /**
     * Sets the gamepad active flag.
     * Used to determine if the robot should actively brake (stop) when no input is detected.
     *
     * @param on True if gamepad input is active, false otherwise.
     */
    public void setGamepad(boolean on) {
        isGamepadOn = on;
    }

    /**
     * Moves the robot relative to the field (Field Centric Drive).
     *
     * @param forward Forward movement (Y axis in field coordinates).
     * @param fun Strafe movement (X axis in field coordinates).
     * @param turn Rotation speed.
     */
    public void moveRobotFieldRelative(double forward, double fun, double turn) {
        // Deadband check to stop drift
        if (Math.abs(forward) < DriveConstants.deadband &&
            Math.abs(fun) < DriveConstants.deadband &&
            Math.abs(turn) < DriveConstants.deadband) {
            stop();
            return;
        }

        pinpoint.update(); // Ensure data is fresh
        
        // Get current robot heading from Pinpoint, adjusting for offset
        double botHeading = pinpoint.getHeading(DriveConstants.angleUnit) - yawOffset;
        
        // Rotate the movement vector by the negative of the robot's heading to transform field-relative input to robot-relative
        // If the robot moves with the heading (Robot Centric behavior in Field Centric mode), 
        // it usually means the heading compensation is inverted. 
        // Trying Positive botHeading here to reverse the compensation direction.
        double rotX = fun * Math.cos(botHeading) - forward * Math.sin(botHeading);
        double rotY = fun * Math.sin(botHeading) + forward * Math.cos(botHeading);

        // Apply strafing balance correction to counteract mechanical inefficiencies in mecanum strafing
        rotX = -rotX * strafingBalance;

        // Normalize motor powers so that the maximum power is 1.0 if any calculated power exceeds 1.0
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        // Set motor powers
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Moves the robot relative to itself (Robot Centric Drive).
     *
     * @param forward Forward speed (positive is forward).
     * @param fun Strafe speed (positive is right, but notice the sign flip logic below).
     * @param turn Rotation speed (positive is turn right).
     */
    public void moveRobot(double forward, double fun, double turn) {
        // Deadband check to stop drift
        if (Math.abs(forward) < DriveConstants.deadband &&
            Math.abs(fun) < DriveConstants.deadband &&
            Math.abs(turn) < DriveConstants.deadband) {
            // Manually set motors to 0 to ensure braking
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            return;
        }

        // Apply strafing balance and sign correction
        // Note: 'fun' input usually assumes positive is right, but standard mecanum math might expect different sign convention
        // Here we invert 'fun' and apply balance
        double rotX = -fun * strafingBalance; 
        double rotY = forward;

        // Normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        // Set motor powers
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Gets the current pose of the robot from the Pinpoint driver.
     *
     * @return The current Pose2D (x, y, heading).
     */
    public Pose2D getPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        return new Pose2D(
            DistanceUnit.INCH, 
            pose.getX(DistanceUnit.INCH), 
            pose.getY(DistanceUnit.INCH), 
            DriveConstants.angleUnit, 
            pose.getHeading(DriveConstants.angleUnit)
        );
    }

    /**
     * Gets the current yaw offset.
     * @return Yaw offset in radians.
     */
    public double getYawOffset() {
        return yawOffset;
    }

    /**
     * Checks if the robot's heading is within a small tolerance of a setpoint.
     *
     * @param headingSetPoint The target heading.
     * @return True if at setpoint, false otherwise.
     */
    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(pinpoint.getHeading(DriveConstants.angleUnit), headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    /**
     * Applies braking by setting motor powers to 0.
     * Relies on ZeroPowerBehavior.BRAKE.
     */
    private void applyBreak() {
        moveRobot(0, 0, 0);
    }

    /**
     * Calculates the angle from the robot's current position to the target position.
     * 
     * @param targetX Target X coordinate in inches.
     * @param targetY Target Y coordinate in inches.
     * @return Angle in radians from current position to target (range: -π to π).
     */
    public double getAngleToTarget(double targetX, double targetY) {
        Pose2D currentPose = getPose();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);
        
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        
        // atan2 returns angle in radians from -π to π
        // This angle points FROM current position TO target
        return Math.atan2(deltaY, deltaX);
    }
    
    /**
     * Calibrates the Pinpoint odometry using vision data from AprilTag detection.
     * Updates X, Y, and heading based on Limelight's field-space position.
     * 
     * @param vision The Vision subsystem to get robot pose from.
     * @param alliance The alliance color (used to set yawOffset for field-centric driving).
     * @return True if calibration was successful (valid vision data), false otherwise.
     */
    public boolean visionCalibrate(Vision vision, Vision.Alliance alliance) {
        Pose3D visionPose = vision.getRobotPose();
        
        if (visionPose == null) {
            return false;  // No valid vision data
        }
        
        // Convert Limelight Pose3D to Pinpoint Pose2D
        Pose2D calibratedPose = Util.visionPoseToPinpointPose(visionPose);
        
        // Update Pinpoint position
        pinpoint.setPosition(calibratedPose);
        
        // Update heading
        pinpoint.setHeading(calibratedPose.getHeading(DriveConstants.angleUnit), DriveConstants.angleUnit);
        
        // Set yaw offset based on alliance for field-centric driving
        // Blue alliance faces 180° (π), Red alliance faces 0°
        yawOffset = (alliance == Vision.Alliance.BLUE) ? Math.PI : 0;
        
        return true;  // Calibration successful
    }

    @Override
    public void periodic() {
        // Update Pinpoint driver to ensure latest data is available
        pinpoint.update();

        // If no gamepad input is detected, apply brakes to hold position
        if (!isGamepadOn) {
            applyBreak();
        }

        // Update last recorded pose
        lastPose = getPose();
    }
}
