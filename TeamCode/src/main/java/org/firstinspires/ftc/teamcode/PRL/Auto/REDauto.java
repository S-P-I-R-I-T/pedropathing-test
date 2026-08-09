package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable // Panels
public class REDauto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private ActionManaging action;
    private LimelightClass limelight;
    private ElapsedTime timer = new ElapsedTime();

    // State machine phases
    private static final int S_ALIGN = 0;      // Outtake spin-up + Limelight align
    private static final int S_FEED = 1;       // Intake feed (shoot)
    private static final int S_WIND_DOWN = 2;  // Outtake reverse until velocity ~0
    private static final int S_DRIVE = 3;      // Follow next path with intake on
    private static final int S_DONE = 4;       // Finished

    private int phase = S_ALIGN;
    private int pathIndex = -1; // -1 = first shoot, then 0..3 for RED1..RED4

    public static final int RED_TAG_ID = 24; // TODO: 레드 얼라이언스 태그 ID로 변경
    public static final int SHOOT_ZONE = 1;  // 1 = far, 2 = near
    public static final double SHOOT_VELOCITY_PERCENT = 0.8;

    public static final double FEED_TIME = 3.0;        // seconds of intake feed per shot

    public static final double WIND_DOWN_TIMEOUT = 2.0; // seconds fallback

    // Limelight turret tracking (MainTeleopBlue 참고)
    public static double kP = 0.02;
    public static double kD = 0.003;
    private double lastError = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        action = new ActionManaging(hardwareMap);
        limelight = new LimelightClass(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        limelight.setTargetTagID(RED_TAG_ID);
        limelight.start();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        limelight.update();
        phase = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Phase", phase);
        panelsTelemetry.debug("Path Index", pathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake Velocity", action.Outtake_Velocity());
        panelsTelemetry.debug("Has Target", limelight.hasTarget());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    public int autonomousPathUpdate() {
        switch (phase) {
            case S_ALIGN:
                // 아웃테이크 돌리고 라임라이트로 얼라인, 목표 속도의 80% 도달 시 발사
                action.Outtake_On(SHOOT_ZONE);
                limelightAlign();
                if (action.Outtake_Velocity() >= targetVelocity() * SHOOT_VELOCITY_PERCENT) {
                    phase = S_FEED;
                    timer.reset();
                }
                break;
            case S_FEED:
                // 인테이크 돌려 발사
                action.Intake_On(1);
                limelightAlign();
                if (timer.seconds() >= FEED_TIME) {
                    action.Intake_Off();
                    phase = S_WIND_DOWN;
                    timer.reset();
                }
                break;
            case S_WIND_DOWN:
                // 아웃테이크 리버스로 속도 0/음수로
                action.Outtake_Reverse();
                if (action.Outtake_Velocity() <= 0 || timer.seconds() >= WIND_DOWN_TIMEOUT) {
                    pathIndex++;
                    if (pathIndex < paths.RED.length) {
                        // 인테이크 켜고 다음 path로
                        action.Intake_On(2);
                        followNextPath();
                        phase = S_DRIVE;
                    } else {
                        phase = S_DONE;
                    }
                }
                break;
            case S_DRIVE:
                // 이동 중 인테이크 수집 + 아웃테이크 리버스 유지
                action.Outtake_Reverse();
                action.Intake_On(2);
                if (!follower.isBusy()) {
                    // 도착하면 인테이크 끄고 다음 발사 준비
                    action.Intake_Off();
                    phase = S_ALIGN;
                }
                break;
            case S_DONE:
            default:
                break;
        }
        return phase;
    }

    private void followNextPath() {
        if (pathIndex == 0) {
            follower.followPath(paths.RED[0], true);
        } else {
            follower.followPath(paths.RED[pathIndex]);
        }
    }

    private double targetVelocity() {
        return SHOOT_ZONE == 1 ? ActionManaging.Shooting_Far_Velocity : ActionManaging.Shooting_Near_Velocity;
    }

    private void limelightAlign() {
        if (limelight.hasTarget()) {
            double error = limelight.getTx();
            double derivative = error - lastError;
            lastError = error;

            double power = kP * error + kD * derivative;
            power = Math.max(-0.6, Math.min(0.6, power));

            action.Turret_SetPower(power);
        } else {
            action.Turret_SetPower(0);
            lastError = 0;
        }
    }

    public static class Paths {
        public PathChain RED1;
        public PathChain RED2;
        public PathChain RED3;
        public PathChain RED4;
        public PathChain[] RED;

        public Paths(Follower follower) {
            RED1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.500, 15.300),
                                    new Pose(76.500, 31.000),
                                    new Pose(119.500, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            RED2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(119.500, 35.000),
                                    new Pose(86.500, 15.300)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();

            RED3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.500, 15.300),
                                    new Pose(85.500, 59.000),
                                    new Pose(119.500, 59.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                    .build();

            RED4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(119.500, 59.000),
                                    new Pose(86.500, 15.300)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                    .build();

            RED = new PathChain[]{RED1, RED2, RED3, RED4};
        }
    }
}
