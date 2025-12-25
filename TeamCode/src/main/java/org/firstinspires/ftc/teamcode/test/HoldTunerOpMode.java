package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/**
 * Hold Tuner OpMode
 * 
 * Tests the position hold (brake) functionality.
 * 
 * Instructions:
 * 1. Open FTC Dashboard and expand HoldTunerOpMode
 * 2. Run this OpMode
 * 3. Push the robot and see if it returns to position
 * 4. Adjust kP_XY and kP_H in Dashboard
 * 
 * Tips:
 * - If robot oscillates: decrease kP
 * - If robot doesn't hold: increase kP
 * - If robot turns wrong way: flip sign of kP_H
 */
@Config
@TeleOp(name = "Hold Tuner", group = "Test")
public class HoldTunerOpMode extends OpMode {

    // Tunable parameters (visible in Dashboard under HoldTunerOpMode)
    public static double kP_XY = 0.02;
    public static double kP_H = -0.8;
    public static boolean holdEnabled = true;

    private GoBildaPinpointDriver pinpoint;
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    
    private Pose2D targetPose;
    private boolean lastAState = false;
    private boolean lastBState = false;

    @Override
    public void init() {
        // Initialize motors directly
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Initialize Pinpoint directly
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "od");
        pinpoint.setOffsets(DriveConstants.xPoseDW, DriveConstants.yPoseDW, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED, 
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.resetPosAndIMU();
        
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        telemetry.addData("Status", "Initialized - waiting for start");
        telemetry.update();
    }
    
    @Override
    public void start() {
        // Set target to current pose when starting
        pinpoint.update();
        targetPose = pinpoint.getPosition();
    }

    @Override
    public void loop() {
        // Update odometry
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();
        
        // A button: Reset target to current position (with debounce)
        if (gamepad1.a && !lastAState) {
            targetPose = currentPose;
        }
        lastAState = gamepad1.a;
        
        // B button: Toggle hold on/off (with debounce)
        if (gamepad1.b && !lastBState) {
            holdEnabled = !holdEnabled;
        }
        lastBState = gamepad1.b;
        
        // Calculate errors
        double errorX = targetPose.getX(DistanceUnit.INCH) - currentPose.getX(DistanceUnit.INCH);
        double errorY = targetPose.getY(DistanceUnit.INCH) - currentPose.getY(DistanceUnit.INCH);
        double errorH = angleWrap(targetPose.getHeading(AngleUnit.RADIANS) - currentPose.getHeading(AngleUnit.RADIANS));
        
        // Apply hold if enabled
        if (holdEnabled) {
            double forward = errorY * kP_XY;
            double strafe = errorX * kP_XY;
            double turn = errorH * kP_H;
            
            // Clamp
            forward = Math.max(-1, Math.min(1, forward));
            strafe = Math.max(-1, Math.min(1, strafe));
            turn = Math.max(-1, Math.min(1, turn));
            
            // Convert to field relative
            double heading = currentPose.getHeading(AngleUnit.RADIANS);
            double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
            double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
            rotX *= DriveConstants.strafingBalance;
            
            // Mecanum kinematics
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            leftFrontMotor.setPower((rotY + rotX + turn) / denominator);
            leftBackMotor.setPower((rotY - rotX + turn) / denominator);
            rightFrontMotor.setPower((rotY - rotX - turn) / denominator);
            rightBackMotor.setPower((rotY + rotX - turn) / denominator);
        } else {
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
        }
        
        // Telemetry
        telemetry.addData("=== HOLD TUNER ===", "");
        telemetry.addData("Hold Enabled", holdEnabled);
        telemetry.addLine();
        
        telemetry.addData("--- Current Pose ---", "");
        telemetry.addData("X", String.format("%.2f in", currentPose.getX(DistanceUnit.INCH)));
        telemetry.addData("Y", String.format("%.2f in", currentPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(currentPose.getHeading(AngleUnit.RADIANS))));
        telemetry.addLine();
        
        telemetry.addData("--- Target Pose ---", "");
        telemetry.addData("Target X", String.format("%.2f in", targetPose.getX(DistanceUnit.INCH)));
        telemetry.addData("Target Y", String.format("%.2f in", targetPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Target H", String.format("%.1f deg", Math.toDegrees(targetPose.getHeading(AngleUnit.RADIANS))));
        telemetry.addLine();
        
        telemetry.addData("--- Errors ---", "");
        telemetry.addData("Error X", String.format("%.2f in", errorX));
        telemetry.addData("Error Y", String.format("%.2f in", errorY));
        telemetry.addData("Error H", String.format("%.1f deg", Math.toDegrees(errorH)));
        telemetry.addLine();
        
        telemetry.addData("--- Tunable (expand HoldTunerOpMode in Dashboard) ---", "");
        telemetry.addData("kP_XY", kP_XY);
        telemetry.addData("kP_H", kP_H);
        telemetry.addLine();
        
        telemetry.addData("Controls", "A=reset target, B=toggle hold");
        
        telemetry.update();
    }
    
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
