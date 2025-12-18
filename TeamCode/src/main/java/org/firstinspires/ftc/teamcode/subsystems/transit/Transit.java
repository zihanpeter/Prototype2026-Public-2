package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem handling the Transit (Feeder) mechanism.
 * Controls the servo that pushes rings/elements into the shooter flywheel.
 */
public class Transit extends SubsystemBase {
    public final Servo transitServo;

    // Default state is DOWN (retracted)
    public TransitState transitState = TransitState.DOWN;

    /**
     * Constructor for Transit.
     * Initializes the transit servo.
     *
     * @param hardwareMap The hardware map.
     */
    public Transit(HardwareMap hardwareMap) {
        transitServo = hardwareMap.get(Servo.class, TransitConstants.transitServoName);
    }

    /**
     * Enum representing the positions of the transit servo.
     */
    public enum TransitState {
        UP(TransitConstants.transitUpPos),   // Engaged/Pushing position
        DOWN(TransitConstants.transitDownPos); // Retracted/Resting position

        final double pos;

        TransitState(double transitPos) {
            pos = transitPos;
        }
    }

    /**
     * Sets the target state for the transit servo.
     * @param transitState The desired TransitState.
     */
    public void setTransitState(TransitState transitState) {
        this.transitState = transitState;
    }

    /**
     * Periodic update method.
     * Updates the servo position to match the current state.
     */
    @Override
    public void periodic() {
        transitServo.setPosition(transitState.pos);
    }
}
