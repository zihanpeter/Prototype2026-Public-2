package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Main TeleOp - Field Centric Driving with Auto-Aim.
 * 
 * Controls:
 * - Left Stick: Move (field-centric)
 * - Right Stick: Rotate
 * - A (hold): Align to goal tag (20/24) in view
 * - B (hold): Spin to search for last aligned tag
 * - Left Stick Click: Reset heading
 */
@Config
@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        // Default drive command
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive,
                robot.vision,
                gamepadEx1, 
                isAuto,
                () -> false  // unused parameter kept for compatibility
        ));

        // Left stick button: Reset heading to 0
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.resetHeading())
        );

        DriverControls.bind(gamepadEx1, robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // --- Pose Telemetry ---
        Pose2D pose = robot.drive.getPose();
        telemetry.addData("X", String.format("%.2f in", pose.getX(DistanceUnit.INCH)));
        telemetry.addData("Y", String.format("%.2f in", pose.getY(DistanceUnit.INCH)));
        telemetry.addData("Heading", String.format("%.1f deg", Math.toDegrees(pose.getHeading(AngleUnit.RADIANS))));
        
        // --- Auto-Aim Status ---
        boolean aPressed = gamepadEx1.getButton(GamepadKeys.Button.A);
        boolean shootPressed = gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
                               gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
                               gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.3;
        boolean shouldAlign = aPressed || shootPressed;
        
        int currentTagId = robot.vision.getDetectedTagId();
        
        telemetry.addLine("=== AUTO-AIM ===");
        
        // Current state
        if (shouldAlign) {
            boolean isGoalTag = (currentTagId == 20 || currentTagId == 24);
            String trigger = aPressed ? "A" : "SHOOT";
            telemetry.addData("Mode", trigger + ": ALIGNING");
            telemetry.addData("Sees Goal Tag", isGoalTag ? "YES (" + currentTagId + ")" : "NO");
            telemetry.addData("tx", String.format("%.2f°", robot.vision.getTx()));
        } else {
            telemetry.addData("Mode", "MANUAL");
            telemetry.addData("Current Tag", currentTagId != -1 ? currentTagId : "None");
        }
        
        // --- Controls ---
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A / Shoot", "Auto-aim to goal tag");
        telemetry.addData("Shoot+Aim", "LB/RB/RT");
        telemetry.addData("LT", "Transit (no aim)");

        telemetry.addData("READY TO SHOOT", robot.shooter.isShooterAtSetPoint());
        telemetry.addData("SHOOTER STATE", robot.shooter.shooterState);
        
        telemetry.update();

        // Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
