package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.shooterServoUpPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Shooter extends SubsystemBase {
    public final DcMotorEx rightShooter;
    public final DcMotorEx leftShooter;
    public final Servo shooterServo;
    public final TelemetryPacket packet = new TelemetryPacket();
    public static boolean readyToShoot = false;
    public final PIDController pidController;
    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        shooterServo = hardwareMap.get(Servo.class, ShooterConstants.shooterServoName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity, ShooterConstants.shooterServoDownPos),
        SLOW(ShooterConstants.slowVelocity, ShooterConstants.shooterServoDownPos),
        MID(ShooterConstants.midVelocity, ShooterConstants.shooterServoDownPos),
        FAST(ShooterConstants.fastVelocity, ShooterConstants.shooterServoUpPos);

        final double shooterVelocity, shooterServoPos;

        ShooterState(double shooterVelocity, double shooterServoPos) {
            this.shooterVelocity = shooterVelocity;
            this.shooterServoPos = shooterServoPos;
        }
    }


    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public double getTargetVelocity() {
        return shooterState.shooterVelocity;
    }

    public boolean isShooterAtSetPoint() {
        // Since velocity is negative, we check if current velocity is less than or equal to target velocity (more negative = faster)
        return rightShooter.getVelocity() <= shooterState.shooterVelocity;
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.STOP) {
            double currentVel = rightShooter.getVelocity();
            double targetVel = shooterState.shooterVelocity;
            double power;

            // Bang-Bang Control with Simple Feedforward Logic
            // Since velocities are negative:
            // currentVel > targetVel means we are SLOWER (e.g. -1000 > -1500) -> Need MORE power (more negative)
            // currentVel <= targetVel means we are FASTER (e.g. -2000 <= -1500) -> Need LESS power (less negative)

            if (currentVel > targetVel) {
                // Too slow, apply max power
                power = 1.0;
            } else {
                // Too fast, reduce power to maintain speed
                // Estimate based on GoBilda 5203-2402-0001 (6000RPM)
                // Max TPS = (6000 / 60) * 28 = 2800 TPS
                // Ratio = target / 2800.0
                power = Math.abs(targetVel) / ShooterConstants.maxVelocityTPS; // 2800 is the max TPS for 6000RPM motor
            }

            // Apply power
            // leftShooter is positive, rightShooter is negative
            leftShooter.setPower(power);
            rightShooter.setPower(-power);

        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        shooterServo.setPosition(shooterState.shooterServoPos);

        if (isShooterAtSetPoint()) {
            readyToShoot = true;
        } else if (rightShooter.getVelocity() > releaseVelocity) {
            readyToShoot = false;
        }

        // packet.put("rightShooterVelocity", rightShooter.getVelocity());
        // FtcDashboard.getInstance().sendTelemetryPacket(packet);
        // packet.put("rightShooterVelocity", rightShooter.getVelocity());
        // FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
