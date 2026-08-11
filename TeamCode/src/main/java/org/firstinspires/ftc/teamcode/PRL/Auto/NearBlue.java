
package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous
@Configurable // Panels
public class NearBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private ActionManaging action;
    private ElapsedTime timer = new ElapsedTime();

    public static final int SHOOT_ZONE = 2;           // 1 = far, 2 = near
    public static final double SHOOT_VELOCITY_PERCENT = 0.8; // 발사 속도의 80% 도달 시 발사
    public static final double FEED_TIME = 3.0;       // seconds of intake feed per shot
    public static final double WIND_DOWN_TIMEOUT = 2.0; // seconds fallback

    private int pathIndex = 0; // 0: goshoot, 1: intake, 2: goshoot2, 3: intake2, 4: goshoot3
    private boolean feeding = false; // 발사(인테이크 피드) 중인지

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        action = new ActionManaging(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21, 123, Math.toRadians(140)));

        paths = new Paths(follower); // Build paths

        follower.followPath(paths.allPaths[0], true); // 1path로 주행 후 발사

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path Index", pathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake Velocity", action.Outtake_Velocity());
        panelsTelemetry.update(telemetry);
    }

    private void autonomousPathUpdate() {
        if (follower.isBusy()) {
            // 이동 중: 아웃테이크는 죽이고, 인테이크 경로일 때만 인테이크2 돌리면서 이동
            action.Outtake_Reverse();
            if (isIntakePath()) {
                action.Intake_On(2);
            } else {
                action.Intake_Off();
            }
            return;
        }

        if (isShootPath()) {
            shoot(); // 도착하면 스핀업 후 인테이크1로 발사
        } else if (pathIndex < paths.allPaths.length - 1) {
            // 인테이크 위치 도착: 인테이크 끄고 다음 발사 위치로
            action.Intake_Off();
            pathIndex++;
            follower.followPath(paths.allPaths[pathIndex]);
        }
    }

    private void shoot() {
        if (!feeding) {
            // 아웃테이크 스핀업
            action.Intake_Off();
            action.Outtake_On(SHOOT_ZONE);
            if (action.Outtake_Velocity() >= targetVelocity() * SHOOT_VELOCITY_PERCENT) {
                // 벨로시티 충분히 올라오면 인테이크1로 3초 발사
                feeding = true;
                timer.reset();
                action.Intake_On(1);
            }
            return;
        }

        if (timer.seconds() >= FEED_TIME) {
            action.Intake_Off();
            action.Outtake_Reverse();
            // 감속 확인 후 다음 경로로
            if (action.Outtake_Velocity() <= 0 || timer.seconds() >= FEED_TIME + WIND_DOWN_TIMEOUT) {
                feeding = false;
                pathIndex++;
                if (pathIndex < paths.allPaths.length) {
                    follower.followPath(paths.allPaths[pathIndex]);
                }
            }
        }
    }

    private boolean isShootPath() {
        return pathIndex % 2 == 0; // 0, 2, 4 = goshoot 계열
    }

    private boolean isIntakePath() {
        return pathIndex % 2 == 1; // 1, 3 = intake 계열
    }

    private double targetVelocity() {
        return SHOOT_ZONE == 1 ? ActionManaging.Shooting_Far_Velocity : ActionManaging.Shooting_Near_Velocity;
    }

    public static class Paths {
        public PathChain goshoot;
        public PathChain intake;
        public PathChain goshoot2;
        public PathChain intake2;
        public PathChain goshoot3;
        public PathChain[] allPaths;

        public Paths(Follower follower) {
            goshoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.000, 123.000),

                                    new Pose(61.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))

                    .build();

            intake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.000, 84.000),
                                    new Pose(52.061, 78.048),
                                    new Pose(20.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            goshoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.000, 85.000),

                                    new Pose(61.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.000, 84.000),
                                    new Pose(77.000, 43.000),
                                    new Pose(16.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))

                    .build();

            goshoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.000, 63.000),

                                    new Pose(61.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))

                    .build();

            allPaths = new PathChain[]{goshoot, intake, goshoot2, intake2, goshoot3};
        }
    }
}
