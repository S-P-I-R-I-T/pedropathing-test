package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.pedropathing.follower.Follower;
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
public class BLUENEAR extends OpMode {
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
        DRIVE_SHOOTPOS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(20, 120, Math.toRadians(144));
    private final Pose shootPose = new Pose(41, 99, Math.toRadians(144));

    private final Pose wp1Pose = new Pose(41, 83, Math.toRadians(180));
    private final Pose gpp1Pose = new Pose(20, 83, Math.toRadians(180));

    private final Pose wp2Pose = new Pose(41, 59, Math.toRadians(180));
    private final Pose gpp2Pose = new Pose(20, 59, Math.toRadians(180));

    private final Pose endPose = new Pose(30, 79, Math.toRadians(144));


    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntakeGPP1, driveIntakeGPP1ShootPos;
    private PathChain driveShootPosIntakeGPP2, driveIntakeGPP2ShootPos;
    private PathChain driveShootPosEndPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        driveShootPosIntakeGPP1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, wp1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(wp1Pose, gpp1Pose))
                .setLinearHeadingInterpolation(wp1Pose.getHeading(), gpp1Pose.getHeading())
                .build();


        driveIntakeGPP1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gpp1Pose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();


        driveShootPosIntakeGPP2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, wp2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(wp2Pose, gpp2Pose))
                .setLinearHeadingInterpolation(wp2Pose.getHeading(), gpp2Pose.getHeading())
                .build();


        driveIntakeGPP2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gpp2Pose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();


        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
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
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3) {
                    action.Intake_On(2);
                    action.Stopper_On();
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

                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;

            case DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
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