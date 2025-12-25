package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * Driver Controls Configuration.
 * Centralizes the gamepad button bindings for the robot.
 */
public class DriverControls {

    /**
     * Binds the gamepad controls to the robot commands.
     *
     * @param gamepad The gamepad wrapper (GamepadEx).
     * @param robot The robot hardware container.
     */
    public static void bind(GamepadEx gamepad, Robot robot) {
        // Reset Field Centric Heading (Left Stick Button)
        // Note: For RobotCentric drive, this just resets odometry, which is harmless.
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> robot.drive.reset(0))
        );

        // Slow Shoot (Left Bumper - Close Shot)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> {
                    robot.shooter.cancelAutoBrakeCycle();  // Cancel brake when pressing
                    robot.shooter.setShooterState(Shooter.ShooterState.SLOW);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    robot.shooter.setShooterState(Shooter.ShooterState.STOP);
                    robot.shooter.startAutoBrakeCycle();  // Start brake when releasing
                })
        );

        // Mid Shoot (Right Bumper - Mid Shot)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> {
                    robot.shooter.cancelAutoBrakeCycle();  // Cancel brake when pressing
                    robot.shooter.setShooterState(Shooter.ShooterState.MID);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    robot.shooter.setShooterState(Shooter.ShooterState.STOP);
                    robot.shooter.startAutoBrakeCycle();  // Start brake when releasing
                })
        );

        // Fast Shoot (Right Trigger - Far Shot)
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TeleOpConstants.slowShootTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> {
                    robot.shooter.cancelAutoBrakeCycle();  // Cancel brake when pressing
                    robot.shooter.setShooterState(Shooter.ShooterState.FAST);
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    robot.shooter.setShooterState(Shooter.ShooterState.STOP);
                    robot.shooter.startAutoBrakeCycle();  // Start brake when releasing
                })
        );

        // Transit Fire (Left Trigger > Threshold)
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TeleOpConstants.transitFireTriggerThreshold
        ).whenHeld(
                new TransitCommand(robot.transit, robot.shooter)
        );

        // Reverse Intake (D-Pad Up)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_UP)
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setReversed(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setReversed(false))
        );

        // Intake Full Power (Left Trigger > Threshold)
        new FunctionalButton(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TeleOpConstants.intakeFullPowerTriggerThreshold
        ).whenHeld(
                new InstantCommand(() -> robot.intake.setFullPower(true))
        ).whenReleased(
                new InstantCommand(() -> robot.intake.setFullPower(false))
        );

        // Manual Brake Control (D-Pad Down)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)
        ).whenHeld(
                new InstantCommand(() -> robot.shooter.manualEngageBrake())
        ).whenReleased(
                new InstantCommand(() -> robot.shooter.manualReleaseBrake())
        );
    }
}

