package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final Servo transitServo;

    public TransitState transitState = TransitState.DOWN;

    public Transit(HardwareMap hardwareMap) {
        transitServo = hardwareMap.get(Servo.class, TransitConstants.transitServoName);
    }


    public enum TransitState {
        UP(TransitConstants.transitUpPos),
        DOWN(TransitConstants.transitDownPos);

        final double pos;

        TransitState(double transitPos) {
            pos = transitPos;
        }
    }

    public void setTransitState(TransitState transitState) {
        this.transitState = transitState;
    }

    @Override
    public void periodic() {
        transitServo.setPosition(transitState.pos);
    }
}
