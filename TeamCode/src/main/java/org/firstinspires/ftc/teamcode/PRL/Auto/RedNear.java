package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class RedNear extends OpMode {
    TelemetryManager panelsTelemetry;
    Follower follower;
    Timer pathtimer, opModeTimer;
    ActionManaging action;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEGPP1,
        DRIVE_INTAKEGPP1_SHOOTPOS,
        SHOOT2,
        DRIVE_SHOOTPOS_INTAKEGPP2,
        DRIVE_INTAKEGPP2_SHOOTPOS,
        SHOOT3,
        DRIVE_SHOOTPOS_INTAKEGPP3A,
        DRIVE_INTAKEGPP3A_INTAKEGPP3B,
        DRIVE_INTAKEGPP3B_SHOOTPOS,
        SHOOT4,
        DRIVE_SHOOTPOS_INTAKEGPP3A_2,
        DRIVE_INTAKEGPP3A_INTAKEGPP3B_2,
        DRIVE_INTAKEGPP3B_SHOOTPOS_2,
        SHOOT5
    }

    PathState pathState;


    private final Pose startPose = new Pose(121.5, 120, Math.toRadians(36));
    private final Pose shootPose = new Pose(100.5, 99, Math.toRadians(36));
    private final Pose gpp1Pose = new Pose(122.5, 82.5, Math.toRadians(0));
    private final Pose gpp2Pose = new Pose(122.5, 59, Math.toRadians(0));
    private final Pose gpp3aPose = new Pose(126.5, 37, Math.toRadians(0));
    private final Pose gpp3bPose = new Pose(129.9, 59, Math.toRadians(31));


    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntakeGPP1, driveIntakeGPP1ShootPos;
    private PathChain driveShootPosIntakeGPP2, driveIntakeGPP2ShootPos;
    private PathChain driveShootPosIntakeGPP3a, driveIntakeGPP3aGPP3b, driveGPP3bShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntakeGPP1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(59, 81), gpp1Pose))
                .setConstantHeadingInterpolation(gpp1Pose.getHeading())
                .build();


        driveIntakeGPP1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gpp1Pose, shootPose))
                .setLinearHeadingInterpolation(gpp1Pose.getHeading(), shootPose.getHeading())
                .build();


        driveShootPosIntakeGPP2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(75.18459302325581, 56.028343023255815), gpp2Pose))
                .setConstantHeadingInterpolation(gpp2Pose.getHeading())
                .build();


        driveIntakeGPP2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gpp2Pose, shootPose))
                .setLinearHeadingInterpolation(gpp2Pose.getHeading(), shootPose.getHeading())
                .build();


        driveShootPosIntakeGPP3a = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gpp3aPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        driveIntakeGPP3aGPP3b = follower.pathBuilder()
                .addPath(new BezierCurve(gpp3aPose, new Pose(9.95261627906976, 50.00726744186045), gpp3bPose))
                .setLinearHeadingInterpolation(gpp3aPose.getHeading(), gpp3bPose.getHeading())
                .build();


        driveGPP3bShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(gpp3bPose, new Pose(28.122674418604653, 46.710029069767444), shootPose))
                .setLinearHeadingInterpolation(gpp3bPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1) {
                    action.Outtake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Preload Shoot");

                    follower.followPath(driveShootPosIntakeGPP1, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP1);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEGPP1:
                action.Intake_On(1);
                action.Stopper_On();
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    action.Stopper_off();
                    panelsTelemetry.debug("Status", "Done Intake GPP1");

                    follower.followPath(driveIntakeGPP1ShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEGPP1_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEGPP1_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1) {
                    action.Outtake_On(2);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                    action.Stopper_On();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot2");

                    follower.followPath(driveShootPosIntakeGPP2, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP2);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEGPP2:
                action.Intake_On(1);
                action.Stopper_On();
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    action.Stopper_off();
                    panelsTelemetry.debug("Status", "Done Intake GPP2");

                    follower.followPath(driveIntakeGPP2ShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEGPP2_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEGPP2_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1) {
                    action.Outtake_On(2);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                    action.Stopper_On();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot3");

                    follower.followPath(driveShootPosIntakeGPP3a, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP3A);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEGPP3A:
                if (!follower.isBusy()) {
                    follower.followPath(driveIntakeGPP3aGPP3b, true);
                    setPathState(PathState.DRIVE_INTAKEGPP3A_INTAKEGPP3B);
                }
                break;

            case DRIVE_INTAKEGPP3A_INTAKEGPP3B:
                action.Intake_On(1);
                action.Stopper_On();
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    action.Stopper_off();
                    panelsTelemetry.debug("Status", "Done Intake GPP3");

                    follower.followPath(driveGPP3bShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEGPP3B_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEGPP3B_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1) {
                    action.Outtake_On(2);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                    action.Stopper_On();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot4");


                    follower.followPath(driveShootPosIntakeGPP3a, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP3A_2);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEGPP3A_2:
                if (!follower.isBusy()) {
                    follower.followPath(driveIntakeGPP3aGPP3b, true);
                    setPathState(PathState.DRIVE_INTAKEGPP3A_INTAKEGPP3B_2);
                }
                break;

            case DRIVE_INTAKEGPP3A_INTAKEGPP3B_2:
                action.Intake_On(1);
                action.Stopper_On();
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    action.Stopper_off();
                    panelsTelemetry.debug("Status", "Done Intake GPP3 (2nd)");

                    follower.followPath(driveGPP3bShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEGPP3B_SHOOTPOS_2);
                }
                break;

            case DRIVE_INTAKEGPP3B_SHOOTPOS_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT5);
                }
                break;

            case SHOOT5:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1) {
                    action.Outtake_On(2);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                    action.Stopper_On();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Auto Complete");
                }
                break;

            default:
                panelsTelemetry.debug("Status", "No State Command");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathtimer.resetTimer();
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathtimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        action = new ActionManaging(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        panelsTelemetry.debug("Path State", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Path Time", pathtimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("opMode Time", opModeTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }
}