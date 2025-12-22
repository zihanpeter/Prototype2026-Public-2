package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * TeleOp for Red Alliance.
 * Field-Centric Driving with Vision telemetry.
 */
@Config
@Configurable
@TeleOp(name = "TeleOp Red", group = "TeleOp")
public class TeleOpRed extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamepadEx1;
    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        robot.drive.setDefaultCommand(new TeleOpDriveCommand(
                robot.drive, 
                gamepadEx1, 
                isAuto,
                () -> gamepadEx1.getButton(GamepadKeys.Button.A)
        ));

        DriverControls.bind(gamepadEx1, robot);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // Vision calibration: Press B to calibrate position using AprilTag
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
            boolean success = robot.drive.visionCalibrate(robot.vision, Vision.Alliance.RED);
            if (success) {
                telemetry.addData("Vision Calibration", "SUCCESS");
            } else {
                telemetry.addData("Vision Calibration", "FAILED - No tag visible");
            }
        }

        Pose2D pose = robot.drive.getPose();

        // --- Position Telemetry ---
        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", pose.getHeading(AngleUnit.RADIANS));
        
        // --- Vision Telemetry ---
        telemetry.addData("Vision Tag ID", robot.vision.getDetectedTagId());
        Vision.Alliance detectedAlliance = robot.vision.getDetectedAlliance();
        telemetry.addData("Vision Alliance", detectedAlliance.toString());
        telemetry.addData("Vision Has Target", robot.vision.hasTarget());
        
        // --- Limelight Debug ---
        telemetry.addData("LL Connected", robot.vision.isConnected());
        telemetry.addData("LL FPS", robot.vision.getFps());
        telemetry.addData("LL Pipeline", robot.vision.getPipelineIndex());
        telemetry.addData("LL Result Valid", robot.vision.isResultValid());
        telemetry.addData("LL Tags Detected", robot.vision.getNumTagsDetected());
        
        // --- Shooter Telemetry ---
        telemetry.addData("Shooter Velocity", robot.shooter.getVelocity());
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelocity());
        
        if (robot.shooter.isShooterAtSetPoint() && robot.shooter.getTargetVelocity() != 0) {
            telemetry.addData("Shooter Status", "TARGET SPEED REACHED");
        }

        telemetry.update();

        // --- Dashboard Packet ---
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", robot.shooter.getVelocity());
        packet.put("IntakeVelocity", robot.intake.getVelocity());
        org.firstinspires.ftc.teamcode.utils.DashboardUtil.drawRobot(packet, pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

