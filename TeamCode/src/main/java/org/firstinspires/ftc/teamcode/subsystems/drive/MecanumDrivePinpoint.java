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
    
    // Offset for heading to allow resetting heading without resetting full odometry (for field-centric drive)
    private double yawOffset;

    // Flag to indicate if gamepad control is active (used for braking logic)
    public boolean isGamepadOn;

    // Last recorded pose
    Pose2D lastPose;
    
    // Flag to track if vision calibration has been performed
    private boolean hasVisionCalibrated = false;
    
    // Auto-aim: Last aligned tag info
    private int lastAlignedTagId = -1;           // Which tag we last aligned to
    private boolean hasLastAlignedTag = false;   // Whether we have a record

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
     * Resets the robot's heading to 0 (convenience method).
     */
    public void resetHeading() {
        reset(0);
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
     * Applies braking - just sets power to 0 and relies on BRAKE mode.
     * Motors are already set to ZeroPowerBehavior.BRAKE in constructor.
     */
    private void applyBreak() {
        // Simply set all motors to 0 power
        // BRAKE mode will provide electromagnetic resistance
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
     * Calibrates the odometry position using vision data from AprilTag detection.
     * DIRECTLY RESETS the Pinpoint odometry position (like Prototype2026-Public).
     * Only calibrates when detecting RED goal (ID 24) or BLUE goal (ID 20) tags.
     * 
     * @param vision The Vision subsystem to get robot pose from.
     * @param alliance The alliance color for yaw offset.
     * @return True if calibration was successful (valid goal tag detected), false otherwise.
     */
    public boolean visionCalibrate(Vision vision, Vision.Alliance alliance) {
        // Only calibrate if we see a goal tag (red or blue), not obelisk tags
        int detectedTagId = vision.getDetectedTagId();
        if (detectedTagId != Vision.BLUE_GOAL_TAG_ID && detectedTagId != Vision.RED_GOAL_TAG_ID) {
            return false;  // Not a goal tag, don't calibrate
        }
        
        Pose3D visionPose = vision.getRobotPose();
        
        if (visionPose == null) {
            return false;  // No valid vision data
        }
        
        // Convert Limelight Pose3D to field coordinates
        Pose2D calibratedPose = Util.visionPoseToPinpointPose(visionPose);
        
        // DIRECTLY SET the Pinpoint position (like Prototype2026-Public)
        pinpoint.setPosition(calibratedPose);
        
        // Set yaw offset based on alliance (for field-centric driving)
        yawOffset = (alliance == Vision.Alliance.BLUE) ? Math.PI : 0;
        
        hasVisionCalibrated = true;
        
        return true;  // Calibration successful
    }
    
    /**
     * Gets the current pose (same as getPose, for compatibility).
     * After vision calibration, this returns the calibrated position.
     */
    public Pose2D getAbsolutePose() {
        return getPose();
    }
    
    /**
     * Checks if vision calibration has been performed.
     * @return True if vision has calibrated at least once.
     */
    public boolean hasVisionCalibrated() {
        return hasVisionCalibrated;
    }
    
    /**
     * Gets the turn power for auto-aim using Limelight's tx value.
     * Simple and direct - no coordinate conversion needed!
     * 
     * @param vision The Vision subsystem.
     * @return Turn power (-1 to 1). Positive = turn right, Negative = turn left.
     */
    /**
     * A button: Align to goal tag currently in view.
     * Only works if seeing tag 20 or 24.
     * Records which tag we aligned to.
     * 
     * @return Turn power (-1 to 1)
     */
    public double getAlignTurnPower(Vision vision) {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (!isGoalTag) {
            return 0;  // No goal tag in view, do nothing
        }
        
        // Record which tag we're aligning to
        lastAlignedTagId = tagId;
        hasLastAlignedTag = true;
        
        // Use tx to align
        double tx = vision.getTx();
        double kP = 0.025;
        double turn = tx * kP;
        return Math.max(-1, Math.min(1, turn));
    }
    
    /**
     * B button: Spin to search for the last aligned tag.
     * Keeps spinning until that specific tag appears in view.
     * 
     * @param vision The vision subsystem
     * @param spinSpeed Speed to spin (positive = right)
     * @return Turn power, or 0 if found the tag or never aligned before
     */
    public double getSearchTurnPower(Vision vision, double spinSpeed) {
        if (!hasLastAlignedTag) {
            return 0;  // Never aligned before, do nothing
        }
        
        int currentTagId = vision.getDetectedTagId();
        
        if (currentTagId == lastAlignedTagId) {
            return 0;  // Found the tag! Stop spinning
        }
        
        // Keep spinning to search
        return spinSpeed;
    }
    
    /**
     * Checks if we found the last aligned tag.
     */
    public boolean foundLastAlignedTag(Vision vision) {
        if (!hasLastAlignedTag) return false;
        return vision.getDetectedTagId() == lastAlignedTagId;
    }
    
    /**
     * Gets the last aligned tag ID.
     */
    public int getLastAlignedTagId() {
        return hasLastAlignedTag ? lastAlignedTagId : -1;
    }
    
    /**
     * Checks if we have a recorded aligned tag.
     */
    public boolean hasLastAlignedTag() {
        return hasLastAlignedTag;
    }
    
    /**
     * Clears the last aligned tag record.
     */
    public void clearLastAlignedTag() {
        hasLastAlignedTag = false;
        lastAlignedTagId = -1;
    }

    @Override
    public void periodic() {
        // Update Pinpoint driver to ensure latest data is available
        pinpoint.update();

        // If no gamepad input is detected, apply brakes to hold position
        if (!isGamepadOn) {
            applyBreak();
        } else {
            // Only update lastPose when there IS input
            // This way, when input stops, we hold the last "active" position
            lastPose = getPose();
        }
    }
}
