package org.firstinspires.ftc.teamcode.PRL.Class;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretClass {

    private final DcMotorEx turretMotor;
    private final LimelightClass limelight;

    private static final double MOTOR_TICKS_PER_REV = 383.6;
    private static final double GEAR_RATIO = 150.0 / 30;
    private static final double TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO;

    private static final double MAX_ANGLE = 540;
    private static final double MIN_ANGLE = -540;

    public static double TurretOffset = 0;
    public static double GoalX = 0;
    public static double GoalY = 0;
    public static double LimelightCorrectionGain = 1.0;

    private double kP = 0.012;
    private double kI = 0;
    private double kD = 0.001;

    private double targetAngle = 0;
    private double continuousAngle = 0;
    private double lastRawAngle = 0;

    private double integral = 0;
    private double lastError = 0;

    public TurretClass(HardwareMap hardwareMap, LimelightClass limelight) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret_R");
        this.limelight = limelight;

        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Pose robotPose) {
        updateAngle();
        cableManagement();

        updateTargetAngle(robotPose);

        double error = targetAngle - continuousAngle;

        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;

        integral += error;

        double derivative = error - lastError;
        lastError = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = Math.max(-1, Math.min(1, power));

        turretMotor.setPower(power);
    }

    private void updateTargetAngle(Pose robotPose) {
        double bearing = Math.toDegrees(Math.atan2(GoalY - robotPose.getY(), GoalX - robotPose.getX()));
        double desired = bearing - Math.toDegrees(robotPose.getHeading()) + TurretOffset;

        if (limelight != null && limelight.hasTarget()) {
            desired += limelight.getTx() * LimelightCorrectionGain;
        }

        while (desired - targetAngle > 180)
            desired -= 360;
        while (desired - targetAngle < -180)
            desired += 360;

        targetAngle = desired;
    }

    private void updateAngle() {
        double rawAngle = (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        double delta = rawAngle - lastRawAngle;

        if (delta > 180)
            delta -= 360;
        if (delta < -180)
            delta += 360;

        continuousAngle += delta;
        lastRawAngle = rawAngle;
    }

    private void cableManagement() {
        if (continuousAngle > MAX_ANGLE) {
            continuousAngle -= 360;
            targetAngle -= 360;
        } else if (continuousAngle < MIN_ANGLE) {
            continuousAngle += 360;
            targetAngle += 360;
        }
    }

    public void setGoal(double x, double y) {
        GoalX = x;
        GoalY = y;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getAngle() {
        return continuousAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getGoalBearing(Pose robotPose) {
        return Math.toDegrees(Math.atan2(GoalY - robotPose.getY(), GoalX - robotPose.getX()));
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}
