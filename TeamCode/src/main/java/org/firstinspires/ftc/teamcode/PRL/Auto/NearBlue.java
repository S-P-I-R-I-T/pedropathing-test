
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
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ActionManaging action;
    private ElapsedTime timer = new ElapsedTime();

    // State machine phases
    private static final int S_DRIVE = 0;      // 인테이크 켜고 이동
    private static final int S_ALIGN = 1;      // 아웃테이크 스핀업 (near zone)
    private static final int S_FEED = 2;       // 인테이크 휠 세게 돌려 발사
    private static final int S_WIND_DOWN = 3;  // 발사 모터 감속 확인 후 이동
    private static final int S_DONE = 4;       // 종료

    public static final int SHOOT_ZONE = 2;           // 1 = far, 2 = near
    public static final double SHOOT_VELOCITY_PERCENT = 0.8; // 발사 속도의 80% 도달 시 발사
    public static final double FEED_TIME = 3.0;        // seconds of intake feed per shot
    public static final double WIND_DOWN_TIMEOUT = 2.0; // seconds fallback

    private int phase = S_DRIVE;
    private int pathIndex = 0; // 0: goshoot, 1: intake, 2: goshoot2, 3: intake2, 4: goshoot3

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        action = new ActionManaging(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        follower.followPath(paths.allPaths[0], true); // 1path로 주행 후 발사

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Path Index", pathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake Velocity", action.Outtake_Velocity());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (phase) {
            case S_DRIVE:
                // 이동 중 인테이크 수집 + 아웃테이크 리버스 유지
                action.Intake_On(2);
                action.Outtake_Reverse();
                if (!follower.isBusy()) {
                    // 도착하면 인테이크 끄고 다음 발사 준비
                    action.Intake_Off();
                    phase = S_ALIGN;
                }
                break;
            case S_ALIGN:
                // 아웃테이크 스핀업, velocity가 충분히 올라오면 발사
                action.Outtake_On(SHOOT_ZONE);
                if (action.Outtake_Velocity() >= targetVelocity() * SHOOT_VELOCITY_PERCENT) {
                    phase = S_FEED;
                    timer.reset();
                }
                break;
            case S_FEED:
                // 인테이크 휠 세게 돌려 발사
                action.Intake_On(2);
                if (timer.seconds() >= FEED_TIME) {
                    action.Intake_Off();
                    phase = S_WIND_DOWN;
                    timer.reset();
                }
                break;
            case S_WIND_DOWN:
                // 발사 모터가 거의 회전하지 않을 때까지 리버스
                action.Outtake_Reverse();
                if (action.Outtake_Velocity() <= 0 || timer.seconds() >= WIND_DOWN_TIMEOUT) {
                    pathIndex++;
                    if (pathIndex < paths.allPaths.length) {
                        // 인테이크 켜고 다음 path로
                        action.Intake_On(2);
                        followNextPath();
                        phase = S_DRIVE;
                    } else {
                        phase = S_DONE;
                    }
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
            follower.followPath(paths.allPaths[0], true);
        } else {
            follower.followPath(paths.allPaths[pathIndex]);
        }
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
