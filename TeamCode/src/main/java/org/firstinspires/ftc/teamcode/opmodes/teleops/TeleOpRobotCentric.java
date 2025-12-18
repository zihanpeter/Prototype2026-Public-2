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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Robot Centric TeleOp Mode.
 * Simplified driving without field orientation logic.
 * Useful for debugging or if localization fails.
 */
@Config
@Configurable
@TeleOp(name = "TeleOp Robot Centric")
public class TeleOpRobotCentric extends CommandOpMode {
    // Robot Container
    private Robot robot;

    // Gamepad
    private GamepadEx gamepadEx1;

    // Flags
    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        // Initialize Robot Hardware
        robot = new Robot(hardwareMap);
        
        // Initialize Gamepad
        gamepadEx1 = new GamepadEx(gamepad1);

        // Robot Centric Drive Command
        // Uses a RunCommand with a lambda to directly call moveRobot()
        // Note: TeleOpDriveCommand is Field Centric, so we don't use it here.
        robot.drive.setDefaultCommand(new com.arcrobotics.ftclib.command.RunCommand(() -> {
            // Standard FTC Robot Centric Mapping:
            // Left Stick Y: Forward/Backward (Inverted because gamepad Y is negative up)
            // Left Stick X: Strafe Left/Right
            // Right Stick X: Turn Left/Right
            robot.drive.moveRobot(-gamepadEx1.getLeftY(), gamepadEx1.getLeftX(), gamepadEx1.getRightX());
        }, robot.drive));

        // --- Controls (Same as Solo) ---
        // Bind gamepad controls using the centralized DriverControls class
        DriverControls.bind(gamepadEx1, robot);

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        // Basic Telemetry
        telemetry.addData("X", robot.drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  robot.drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", robot.drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset", robot.drive.getYawOffset());
        telemetry.addData("Shooter Current Velocity", robot.shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity", robot.shooter.getTargetVelocity());
        telemetry.addData("Intake Velocity", robot.intake.getVelocity());

        if (robot.shooter.isShooterAtSetPoint() && robot.shooter.getTargetVelocity() != 0) {
            telemetry.addData("Shooter Status", "TARGET SPEED REACHED");
        }
        
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        telemetry.addData("LF Power: ", robot.drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", robot.drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", robot.drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", robot.drive.rightBackMotor.getPower());
        telemetry.addData("Is Gamepad On: ", robot.drive.isGamepadOn);
        telemetry.update();

        // Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", robot.shooter.getVelocity());
        packet.put("IntakeVelocity", robot.intake.getVelocity());
        double targetIntakePower = Intake.isShooting ? 
                org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.transitPower : 
                org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.intakePower;
        packet.put("IntakeTargetPower", targetIntakePower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
