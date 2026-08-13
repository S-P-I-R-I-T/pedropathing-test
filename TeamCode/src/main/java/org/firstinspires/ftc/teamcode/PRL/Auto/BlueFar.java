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
public class BlueFar extends OpMode {
    TelemetryManager panelsTelemetry;
    Follower follower;
    Timer pathtimer, opModeTimer;
    ActionManaging action;

    public enum PathState {
        DRIVE_STARTPOS_PPG,
        DRIVE_PPG1_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEPPG,
        DRIVE_INTAKEPPG_SHOOTPOS,
        SHOOT2,
        DRIVE_SHOOTPOS_INTAKEPGP,
        DRIVE_INTAKEPGP_SHOOTPOS,
        SHOOT3,
        DRIVE_SHOOTPOS_INTAKEPGP2,
        DRIVE_INTAKEPGP2_SHOOTPOS,
        SHOOT4
    }

    PathState pathState;

    private final Pose startPose  = new Pose(56.87, 18.85, Math.toRadians(90));
    private final Pose gppPose   = new Pose(23.97, 34.89, Math.toRadians(180));
    private final Pose shootPose  = new Pose(56.75, 18.18, Math.toRadians(90));
    private final Pose intakePose = new Pose(11.32, 58.28, Math.toRadians(120));

    private PathChain driveStartPosPPG;
    private PathChain driveGPP1ShootPos;
    private PathChain driveShootPosIntakePPG, driveIntakePPG1ShootPos;
    private PathChain driveShootPosIntakePGP, driveIntakeGPP2ShootPos;
    private PathChain driveShootPosIntakePGP2, driveIntakeGPP3ShootPos;

    public void buildPaths() {
        driveStartPosPPG = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(41.67, 34.51, Math.toRadians(180))))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(41.67, 34.51, Math.toRadians(180)), gppPose))
                .setConstantHeadingInterpolation(gppPose.getHeading())
                .build();

        driveGPP1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gppPose, shootPose))
                .setLinearHeadingInterpolation(gppPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(15.78, 63.72, Math.toRadians(90))))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(new BezierLine(new Pose(15.78, 63.72, Math.toRadians(90)), new Pose(13.89, 26.48, Math.toRadians(90))))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(new BezierLine(new Pose(13.89, 26.48, Math.toRadians(90)), new Pose(11.12, 26.35, Math.toRadians(120))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .addPath(new BezierLine(new Pose(11.12, 26.35, Math.toRadians(120)), intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();

        driveIntakePPG1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, new Pose(56.88, 18.40, Math.toRadians(90))))
                .setLinearHeadingInterpolation(intakePose.getHeading(), Math.toRadians(90))
                .build();

        driveShootPosIntakePGP  = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.88, 18.40, Math.toRadians(90)), new Pose(10.52, 59.06, Math.toRadians(132))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                .build();

        driveIntakeGPP2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10.52, 59.06, Math.toRadians(132)), new Pose(56.58, 18.23, Math.toRadians(90))))
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(90))
                .build();

        driveShootPosIntakePGP2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.58, 18.23, Math.toRadians(90)), new Pose(10.93, 59.41, Math.toRadians(132))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                .build();

        driveIntakeGPP3ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(10.93, 59.41, Math.toRadians(132)), new Pose(57.08, 18.57, Math.toRadians(90))))
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(90))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_PPG:
                if (!follower.isBusy()) {
                    follower.followPath(driveGPP1ShootPos, true);
                    setPathState(PathState.DRIVE_PPG1_SHOOTPOS);
                }
                break;

            case DRIVE_PPG1_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (pathtimer.getElapsedTimeSeconds() >= 1 && pathtimer.getElapsedTimeSeconds() < 3) {
                    action.Outtake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 3 && pathtimer.getElapsedTimeSeconds() < 5) {
                    action.Intake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Preload Shoot");

                    follower.followPath(driveShootPosIntakePPG, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPPG);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEPPG:
                action.Intake_On(1);
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Intake GPP1");

                    follower.followPath(driveIntakePPG1ShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEPPG_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEPPG_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT2);
                }
                break;

            case SHOOT2:
                if (pathtimer.getElapsedTimeSeconds() >= 1 && pathtimer.getElapsedTimeSeconds() < 3) {
                    action.Outtake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 3 && pathtimer.getElapsedTimeSeconds() < 5) {
                    action.Intake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot2");

                    follower.followPath(driveShootPosIntakePGP, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPGP);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEPGP:
                action.Intake_On(1);
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Intake GPP2");

                    follower.followPath(driveIntakeGPP2ShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEPGP_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEPGP_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT3);
                }
                break;

            case SHOOT3:
                if (pathtimer.getElapsedTimeSeconds() >= 1 && pathtimer.getElapsedTimeSeconds() < 3) {
                    action.Outtake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 3 && pathtimer.getElapsedTimeSeconds() < 5) {
                    action.Intake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot3");

                    follower.followPath(driveShootPosIntakePGP2, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEPGP2);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEPGP2:
                action.Intake_On(1);
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Intake GPP3");

                    follower.followPath(driveIntakeGPP3ShootPos, true);
                    setPathState(PathState.DRIVE_INTAKEPGP2_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKEPGP2_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT4);
                }
                break;

            case SHOOT4:
                if (pathtimer.getElapsedTimeSeconds() >= 1 && pathtimer.getElapsedTimeSeconds() < 3) {
                    action.Outtake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 3 && pathtimer.getElapsedTimeSeconds() < 5) {
                    action.Intake_On(2);
                }
                if (pathtimer.getElapsedTimeSeconds() >= 5) {
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

        pathtimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        action = new ActionManaging(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_PPG);
        follower.followPath(driveStartPosPPG, true);
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