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
            GoBildaPinpointDriver.EncoderDirection.FORWARD, 
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        
        // Reset Position and IMU on initialization
        pinpoint.resetPosAndIMU();

        // Motor Directions
        // Updated based on user feedback: Forward/Strafe were reversed, Turn was correct.
        // Inverting all motors corrects Forward/Strafe while maintaining relative Turn direction.
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        pinpoint.update(); // Ensure data is fresh
        
        // Get current robot heading from Pinpoint, adjusting for offset
        double botHeading = pinpoint.getHeading(DriveConstants.angleUnit) - yawOffset;
        
        // Rotate the movement vector by the negative of the robot's heading to transform field-relative input to robot-relative
        double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

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
