package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for TeleOp driving with auto-aim support.
 * 
 * A (hold) or any shoot button (LB/RB/RT/LT): Auto-align to goal tag (20/24)
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final Vision vision;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;
    
    // Trigger threshold for shoot buttons
    private static final double TRIGGER_THRESHOLD = 0.3;

    public TeleOpDriveCommand(MecanumDrivePinpoint drive, Vision vision, GamepadEx gamepadEx, 
                              boolean[] isAuto, java.util.function.BooleanSupplier unused) {
        this.drive = drive;
        this.vision = vision;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }
    
    /**
     * Checks if any shoot button (with auto-aim) is pressed.
     * LB = Slow, RB = Mid, RT = Fast (LT excluded - no auto-aim for transit)
     */
    private boolean isShootButtonPressed() {
        return gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER) ||
               gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER) ||
               gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= TRIGGER_THRESHOLD;
    }

    @Override
    public void execute() {
        if (!isAuto[0]) {
            // Get raw inputs
            double rawLeftX = gamepadEx.getLeftX();
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // Check button states
            boolean aPressed = gamepadEx.getButton(GamepadKeys.Button.A);
            boolean shootPressed = isShootButtonPressed();
            
            // Auto-aim triggers: A button OR any shoot button
            boolean shouldAlign = aPressed || shootPressed;
            
            // D-Pad rotation input (always available)
            double dpadTurn = 0;
            if (gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                dpadTurn = -DriveConstants.dpadTurnSpeed;
            } else if (gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                dpadTurn = DriveConstants.dpadTurnSpeed;
            }
            
            // Check for input
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband ||
                               dpadTurn != 0 ||
                               shouldAlign;
            
            if (hasInput) {
                drive.setGamepad(true);
                
                // Apply squared input curve
                double forward = rawLeftY * Math.abs(rawLeftY);
                double strafe = -rawLeftX * Math.abs(rawLeftX);
                
                // Determine turn input
                double turn = rawRightX * Math.abs(rawRightX);  // Manual always works
                turn += dpadTurn;
                
                if (shouldAlign) {
                    // A or Shoot buttons: Add auto-aim to manual turn
                    turn += drive.getAlignTurnPower(vision);
                }
                
                // Clamp turn to [-1, 1]
                turn = Math.max(-1, Math.min(1, turn));
                
                // Drive Field Relative
                drive.moveRobotFieldRelative(forward, strafe, turn);
            }
            else {
                drive.setGamepad(false);
            }
        }
    }
}
