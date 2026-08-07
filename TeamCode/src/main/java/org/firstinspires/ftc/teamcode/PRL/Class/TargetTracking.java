package org.firstinspires.ftc.teamcode.PRL.Class;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TargetTracking {

    private final LimelightClass limelight;
    private final ActionManaging action;

    public static double kP = 0.015;
    public static double kD = 0.003;
    public static double AlignThreshold = 1.0;

    public static double SweepSpeed = 0.2;
    public static double SweepLimit = 45;
    public static double SweepAngleMargin = 3;

    private static final double MOTOR_TICKS_PER_REV = 383.6;
    private static final double GEAR_RATIO = 150.0 / 24.0;
    private static final double TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO;

    private double lastError = 0;

    private boolean sweeping = false;
    private double sweepDirection = 1;
    private double sweepCenter = 0;
    private double sweepTarget = 0;

    private double continuousAngle = 0;
    private double lastRawAngle = 0;

    public TargetTracking(LimelightClass limelight, ActionManaging action) {
        this.limelight = limelight;
        this.action = action;
    }

    public void update(Telemetry telemetry, boolean shooting) {
        limelight.update();
        updateAngle();

        if (!limelight.hasTarget()) {
            sweep(telemetry);
            return;
        }

        if (sweeping) {
            sweeping = false;
            lastError = 0;
        }

        if (shooting && Math.abs(limelight.getTx()) < AlignThreshold) {
            action.Turret_Lock();
            telemetry.addLine("Turret Locked");
            return;
        }

        double error = limelight.getTx();

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kD * derivative;

        action.Turret_SetPower(power);

        telemetry.addData("tx", error);
        telemetry.addData("Power", power);
    }

    private void updateAngle() {
        double rawAngle = (action.Turret_Position() / TICKS_PER_REV) * 360.0;
        double delta = rawAngle - lastRawAngle;

        if (delta > 180)
            delta -= 360;
        if (delta < -180)
            delta += 360;

        continuousAngle += delta;
        lastRawAngle = rawAngle;
    }

    private void sweep(Telemetry telemetry) {
        if (!sweeping) {
            sweeping = true;
            sweepCenter = continuousAngle;
            sweepTarget = sweepCenter + sweepDirection * SweepLimit;
        }

        double error = sweepTarget - continuousAngle;

        if (Math.abs(error) < SweepAngleMargin) {
            sweepDirection *= -1;
            sweepTarget = sweepCenter + sweepDirection * SweepLimit;
        }

        double power = Math.max(-SweepSpeed, Math.min(SweepSpeed, error * 0.01));
        action.Turret_SetPower(power);

        telemetry.addLine("No Target - Sweeping");
        telemetry.addData("Angle", "%.1f", continuousAngle);
        telemetry.addData("SweepTarget", "%.1f", sweepTarget);
    }
}
