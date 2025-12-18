package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_h;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.kP_xy;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.strafingBalance;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Subsystem for the Mecanum Drive using the SparkFun Optical Tracking Odometry Sensor (OTOS).
 * <p>
 * This subsystem handles:
 * <ul>
 *     <li>Hardware initialization for drive motors and OTOS.</li>
 *     <li>Field-centric and Robot-centric driving logic.</li>
 *     <li>Odometry updates via OTOS.</li>
 *     <li>Basic autonomous turning and breaking functions.</li>
 * </ul>
 */
@Config
public class MecanumDriveOTOS extends SubsystemBase {
    public final DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final SparkFunOTOS otos;
    private double yawOffset; // Offset for resetting the robot's heading (in radians)

    public boolean isGamepadOn;

    Pose2D lastPose;

    /**
     * Constructor for MecanumDriveOTOS.
     * Initializes motors, sensors, and configures the OTOS parameters.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public MecanumDriveOTOS(final HardwareMap hardwareMap) {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        // Initialize OTOS sensor
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        isGamepadOn = false;

        // Set zero power behavior to BRAKE for better control
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure OTOS settings
        otos.resetTracking();
        otos.setAngularUnit(DriveConstants.angleUnit);
        otos.setLinearUnit(DriveConstants.distanceUnit);
        otos.setOffset(new SparkFunOTOS.Pose2D(DriveConstants.xPoseOTOS,
                DriveConstants.yPoseOTOS, DriveConstants.headingPoseOTOS));

        // Set motor directions (Reverse left side for mecanum drive)
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPose = new Pose2D(DriveConstants.distanceUnit, 0, 0, DriveConstants.angleUnit, 0);
    }

    /**
     * Stops the robot by setting all motor powers to zero.
     */
    public void stop() {
        moveRobot(0, 0, 0);
    }

    /**
     * Resets the robot's heading to a specified value.
     * Useful for re-calibrating the field-centric drive.
     *
     * @param heading The new heading value in radians.
     */
    public void reset(double heading) {
        yawOffset = otos.getPosition().h + heading;
    }

    /**
     * Sets whether the gamepad is currently controlling the drive.
     *
     * @param on True if gamepad control is active, false otherwise.
     */
    public void setGamepad(boolean on) {
        isGamepadOn = on;
    }

    /**
     * Moves the robot using Field-Centric control.
     * The robot moves relative to the field, regardless of its current orientation.
     *
     * @param forward Forward/Backward component (-1.0 to 1.0).
     * @param fun     Strafe Left/Right component (-1.0 to 1.0).
     * @param turn    Rotation component (-1.0 to 1.0).
     */
    public void moveRobotFieldRelative(double forward, double fun, double turn) {
        // Calculate the robot's heading relative to the reset offset
        double botHeading = otos.getPosition().h - yawOffset;

        // Rotate the movement vector by the negative of the robot's heading
        double rotX = fun * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = fun * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        rotX = rotX * strafingBalance; // Compensate for imperfect strafing mechanics

        // Normalize motor powers so no value exceeds 1.0, preserving the direction vector
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Moves the robot using Robot-Centric control.
     * The robot moves relative to itself.
     *
     * @param forward Forward/Backward component (-1.0 to 1.0).
     * @param fun     Strafe Left/Right component (-1.0 to 1.0).
     * @param turn    Rotation component (-1.0 to 1.0).
     */
    public void moveRobot(double forward, double fun, double turn) {
        double rotX = fun * strafingBalance; // Compensate for imperfect strafing mechanics
        double rotY = forward;

        // Normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    /**
     * Turns the robot by a specified relative angle.
     * Blocking method.
     *
     * @param angle The angle to turn in radians.
     * @param power The motor power to use for turning.
     */
    public void turnRobot(double angle, double power) {
        double preAngle = otos.getPosition().h;
        // Simple blocking loop to turn
        while (otos.getPosition().h - preAngle < angle) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    /**
     * Turns the robot to a specified absolute heading.
     * Blocking method.
     *
     * @param angle The target heading in radians.
     * @param power The motor power to use for turning.
     */
    public void turnRobotTo(double angle, double power) {
        double heading = otos.getPosition().h;
        double needs = (angle - heading) % (2 * Math.PI);
        while (Util.epsilonEqual(angle, otos.getPosition().h, 0.02)) {
            leftFrontMotor.setPower(power * 0.2);
            leftBackMotor.setPower(power * 0.2);
            rightFrontMotor.setPower(power * -0.2);
            rightBackMotor.setPower(power * -0.2);
        }
    }

    /**
     * Gets the current robot pose from the OTOS sensor.
     *
     * @return The current Pose2D (x, y, heading).
     */
    public Pose2D getPose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new Pose2D(DriveConstants.distanceUnit, pose.x, pose.y, DriveConstants.angleUnit, pose.h);
    }

    /**
     * Gets the current yaw offset used for field-centric drive.
     *
     * @return The yaw offset in radians.
     */
    public double getYawOffset() {
        return yawOffset;
    }

    /**
     * Checks if the robot's heading is within a small epsilon of the target setpoint.
     *
     * @param headingSetPoint The target heading.
     * @return True if the heading is close enough, false otherwise.
     */
    public boolean isHeadingAtSetPoint(double headingSetPoint) {
        return Util.epsilonEqual(otos.getPosition().h, headingSetPoint,
                DriveConstants.headingEpsilon);
    }

    /**
     * Applies a simple PID-like braking force to hold the robot's position.
     * Calculates error from the last recorded pose and applies corrective power.
     */
    private void applyBreak() {
        Pose2D p = getPose();

        double errorX = lastPose.getX(DriveConstants.distanceUnit) - p.getX(DriveConstants.distanceUnit);
        double errorY = lastPose.getY(DriveConstants.distanceUnit) - p.getY(DriveConstants.distanceUnit);
        double errorH = angleWrap(lastPose.getHeading(DriveConstants.angleUnit) - p.getHeading(DriveConstants.angleUnit));

        double forward = errorY * kP_xy;
        double strafe  = errorX * kP_xy;
        double turn    = errorH * kP_h;
        
        forward = clip(forward, -1, 1);
        strafe  = clip(strafe,  -1, 1);
        turn    = clip(turn,    -1, 1);

        moveRobotFieldRelative(forward, strafe, turn);
    }

    /**
     * Periodic method called every loop.
     * Updates the last known pose.
     * Uncomment applyBreak() call to enable active braking when gamepad is not in use.
     */
    @Override
    public void periodic() {
        // if (!isGamepadOn) {
        //     applyBreak();
        // }

        lastPose = getPose();
    }
}
