package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.PRL.Class.TargetTracking;
import org.firstinspires.ftc.teamcode.PRL.Class.TurretClass;
import org.firstinspires.ftc.teamcode.PRL.Teleop.MainTeleopBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "MainAutoBlue")
public class MainAutoBlue extends LinearOpMode {

    Follower follower;
    ActionManaging action;
    TurretClass turret;
    TargetTracking tracking;
    LimelightClass limelight;

    public static final boolean UseTurretClass = true;

    public static final double START_X = 60;
    public static final double START_Y = 9;
    public static final double START_HEADING = Math.toRadians(90);

    public static final int PRELOAD_RINGS = 3;
    public static final int SHOT_RINGS = 3;
    public static final int SHOOT_ZONE = 1;

    public static final double RING_FEED_TIME = 400;
    public static final double RING_GAP_TIME = 400;

    int shootState = 0;
    int shootZone = 1;
    int shootRingsLeft = 0;
    long shootTimer = 0;

    @Override
    public void runOpMode() {

        action = new ActionManaging(hardwareMap);

        if (UseTurretClass) {
            turret = new TurretClass(hardwareMap, null);
            turret.setGoal(MainTeleopBlue.GOAL_X, MainTeleopBlue.GOAL_Y);
        } else {
            limelight = new LimelightClass(hardwareMap);
            limelight.setTargetTagID(MainTeleopBlue.BLUE_TAG_ID);
            limelight.start();
            tracking = new TargetTracking(limelight, action);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));

        PathChain chain = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(60.000, 9.000),
                        new Pose(63.000, 27.000),
                        new Pose(15.000, 36.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addTemporalCallback(0, () -> {
                    action.Intake_On();
                    action.Stopper_On();
                })
                .addParametricCallback(1.0, () -> {
                    action.Intake_Off();
                    action.Outtake_On(SHOOT_ZONE);
                })
                .addPath(
                    new BezierLine(
                        new Pose(15.000, 36.000),
                        new Pose(60.000, 9.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addParametricCallback(1.0, () -> startShoot(SHOOT_ZONE, SHOT_RINGS))
                .addPath(
                    new BezierLine(
                        new Pose(60.000, 9.000),
                        new Pose(9.000, 9.000)
                    )
                )
                .setTangentHeadingInterpolation()
                .addTemporalCallback(0, () -> {
                    action.Intake_On();
                    action.Stopper_On();
                })
                .addParametricCallback(1.0, () -> {
                    action.Intake_Off();
                    action.Outtake_On(SHOOT_ZONE);
                })
                .addPath(
                    new BezierLine(
                        new Pose(9.000, 9.000),
                        new Pose(60.000, 9.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addParametricCallback(1.0, () -> startShoot(SHOOT_ZONE, SHOT_RINGS))
                .build();

        waitForStart();

        startShoot(SHOOT_ZONE, PRELOAD_RINGS);

        while (opModeIsActive() && shooting()) {
            follower.update();
            aim();
            updateShoot();
            telemetry.update();
        }

        follower.followPath(chain);

        while (opModeIsActive() && (follower.isBusy() || shooting())) {
            follower.update();
            aim();
            updateShoot();

            telemetry.addData("Pose", follower.getPose());
            telemetry.addData("Shooter", "%.0f", action.Outtake_Velocity());
            telemetry.addData("ShootState", shootState);
            telemetry.addData("RingsLeft", shootRingsLeft);

            telemetry.update();
        }

        action.Intake_Off();
        action.Outtake_Off();
        if (limelight != null) limelight.stop();

        PoseHolder.endPose = follower.getPose();
    }

    void aim() {
        if (UseTurretClass) {
            turret.update(follower.getPose());
        } else {
            tracking.update(telemetry, shooting());
        }
    }

    void startShoot(int zone, int rings) {
        shootZone = zone;
        shootRingsLeft = rings;
        shootState = 1;
        action.Outtake_On(zone);
    }

    void updateShoot() {
        if (shootState == 0) return;

        long now = System.currentTimeMillis();
        double targetVelocity = shootZone == 1 ? ActionManaging.Shooting_Far_Velocity : ActionManaging.Shooting_Near_Velocity;

        switch (shootState) {
            case 1:
                if (action.Outtake_Velocity() >= targetVelocity * 0.98) {
                    shootState = 2;
                    shootTimer = now;
                }
                break;
            case 2:
                action.Intake_On();
                if (now - shootTimer > RING_FEED_TIME) {
                    action.Intake_Off();
                    shootRingsLeft--;
                    if (shootRingsLeft <= 0) {
                        shootState = 0;
                    } else {
                        shootState = 3;
                        shootTimer = now;
                    }
                }
                break;
            case 3:
                if (now - shootTimer > RING_GAP_TIME) {
                    shootState = 2;
                    shootTimer = now;
                }
                break;
        }
    }

    boolean shooting() {
        return shootState != 0;
    }
}
