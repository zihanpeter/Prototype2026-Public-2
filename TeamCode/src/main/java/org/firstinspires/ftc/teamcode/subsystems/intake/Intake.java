package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    public final DcMotorEx intakeMotor;
    public final TelemetryPacket packet = new TelemetryPacket();

    public static boolean isRunning = false;
    public static boolean isShooting = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorName);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        isRunning = true; // Start running by default
    }

    public void toggle() {
        isRunning = !isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void toggleShooting() {
        isShooting = !isShooting;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    @Override
    public void periodic() {
        if (isRunning) {
            double targetPower = isShooting ? IntakeConstants.transitPower : IntakeConstants.intakePower;

            /*
            // Jamming protection: Reduce power if speed drops too low while running
            // 20% of max theoretical speed
            double threshold = IntakeConstants.maxVelocityTPS * IntakeConstants.jammingThresholdRatio;
            double currentVel = Math.abs(intakeMotor.getVelocity());

            // Only apply if we are trying to move fast (target > 0.5) to avoid triggering on slow moves if any
            if (Math.abs(targetPower) > IntakeConstants.jammingPower && currentVel < threshold) {
                targetPower = Math.signum(targetPower) * IntakeConstants.jammingPower;
            }
            */

            intakeMotor.setPower(targetPower);
        }
        else {
            intakeMotor.setPower(0);
        }

        // packet.put("IntakeVelocity", getVelocity());
        // FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
