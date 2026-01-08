package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.opmodes.teleops.TeleOpConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
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
     * @param isAuto Flag array to disable chassis when adaptive shooting is active.
     */
    public static void bind(GamepadEx gamepad, Robot robot, boolean[] isAuto) {
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

        // Transit Fire (Left Trigger + Shoot Button)
        // Only fires when BOTH left trigger AND a shoot button (LB/RB/RT) are pressed
        new FunctionalButton(
                () -> {
                    boolean leftTriggerPressed = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= TeleOpConstants.transitFireTriggerThreshold;
                    boolean shootButtonPressed = gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
                                                  gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
                                                  gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TeleOpConstants.slowShootTriggerThreshold;
                    return leftTriggerPressed && shootButtonPressed;
                }
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
        
        // ==================== ADAPTIVE FIRE (X = Blue Goal, B = Red Goal) ====================
        // X Button: Calculate distance to BLUE goal (tag 20), set adaptive velocity/servo, fire
        // B Button: Calculate distance to RED goal (tag 24), set adaptive velocity/servo, fire
        // NOTE: Adaptive shooting ONLY works when a Goal Tag (ID 20 or 24) is visible!
        
        // X Button - BLUE Goal (only if seeing Goal Tag)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.X)
        ).whenHeld(
                new InstantCommand(() -> {
                    int currentTag = robot.vision.getDetectedTagId();
                    boolean isGoalTag = (currentTag == Vision.BLUE_GOAL_TAG_ID || currentTag == Vision.RED_GOAL_TAG_ID);
                    
                    if (isGoalTag) {
                        isAuto[0] = true;  // Disable chassis during adaptive shooting
                        robot.drive.stop();  // Stop chassis immediately
                        robot.shooter.cancelAutoBrakeCycle();  // Cancel brake
                        // Calculate adaptive velocity and servo position based on distance to BLUE goal
                        double adaptiveVelocity = robot.drive.calculateAdaptiveVelocity(Vision.BLUE_GOAL_TAG_ID);
                        double adaptiveServoPos = robot.drive.calculateAdaptiveServoPosition(Vision.BLUE_GOAL_TAG_ID);
                        robot.shooter.setAdaptiveVelocity(adaptiveVelocity);
                        robot.shooter.setAdaptiveServoPosition(adaptiveServoPos);
                    }
                    // If no Goal Tag visible, do nothing (adaptive disabled)
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    isAuto[0] = false;  // Re-enable chassis
                    robot.shooter.setShooterState(Shooter.ShooterState.STOP);
                    robot.shooter.startAutoBrakeCycle();  // Start brake when releasing
                })
        );
        
        // Fire when X is held (only if seeing Goal Tag)
        new FunctionalButton(
                () -> {
                    if (!gamepad.getButton(GamepadKeys.Button.X)) return false;
                    int currentTag = robot.vision.getDetectedTagId();
                    return (currentTag == Vision.BLUE_GOAL_TAG_ID || currentTag == Vision.RED_GOAL_TAG_ID);
                }
        ).whenHeld(
                new TransitCommand(robot.transit, robot.shooter)
        );
        
        // B Button - RED Goal (only if seeing Goal Tag)
        new FunctionalButton(
                () -> gamepad.getButton(GamepadKeys.Button.B)
        ).whenHeld(
                new InstantCommand(() -> {
                    int currentTag = robot.vision.getDetectedTagId();
                    boolean isGoalTag = (currentTag == Vision.BLUE_GOAL_TAG_ID || currentTag == Vision.RED_GOAL_TAG_ID);
                    
                    if (isGoalTag) {
                        isAuto[0] = true;  // Disable chassis during adaptive shooting
                        robot.drive.stop();  // Stop chassis immediately
                        robot.shooter.cancelAutoBrakeCycle();  // Cancel brake
                        // Calculate adaptive velocity and servo position based on distance to RED goal
                        double adaptiveVelocity = robot.drive.calculateAdaptiveVelocity(Vision.RED_GOAL_TAG_ID);
                        double adaptiveServoPos = robot.drive.calculateAdaptiveServoPosition(Vision.RED_GOAL_TAG_ID);
                        robot.shooter.setAdaptiveVelocity(adaptiveVelocity);
                        robot.shooter.setAdaptiveServoPosition(adaptiveServoPos);
                    }
                    // If no Goal Tag visible, do nothing (adaptive disabled)
                })
        ).whenReleased(
                new InstantCommand(() -> {
                    isAuto[0] = false;  // Re-enable chassis
                    robot.shooter.setShooterState(Shooter.ShooterState.STOP);
                    robot.shooter.startAutoBrakeCycle();  // Start brake when releasing
                })
        );
        
        // Fire when B is held (only if seeing Goal Tag)
        new FunctionalButton(
                () -> {
                    if (!gamepad.getButton(GamepadKeys.Button.B)) return false;
                    int currentTag = robot.vision.getDetectedTagId();
                    return (currentTag == Vision.BLUE_GOAL_TAG_ID || currentTag == Vision.RED_GOAL_TAG_ID);
                }
        ).whenHeld(
                new TransitCommand(robot.transit, robot.shooter)
        );
    }
}

