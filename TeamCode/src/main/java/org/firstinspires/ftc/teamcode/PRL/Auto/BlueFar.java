package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.HoodControl;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BlueFar extends OpMode {
    TelemetryManager panelsTelemetry;
    Follower follower;
    Timer pathtimer, opModeTimer;
    ActionManaging action;
    HoodControl hood;

    public static double kP = 0.02;
    public static double kD = 0.003;
    double lastError = 0;
    LimelightClass limelight;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEGPP1,
        DRIVE_INTAKEGPP1INTAKE,
        DRIVE_INTAKEGPP1_SHOOTPOS,
        SHOOT2,
        DRIVE_SHOOTPOS_LEAVEPOS;
    }

    PathState pathState;

    private final Pose startPose = new Pose(55, 8, Math.toRadians(180));
    private final Pose shootPose = new Pose(55, 36, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(55, 36, Math.toRadians(180));

    private final Pose wp1Pose = new Pose(10, 36, Math.toRadians(180));
    private final Pose gpp1Pose = new Pose(55, 15, Math.toRadians(180));


    private final Pose leavePose = new Pose(50,90,Math.toRadians(0));



    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntakeGPP1, driveIntakeGPP1ShootPos;
    private PathChain driveShootPosLeave,driveIntakeGPP1Intake;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        driveShootPosIntakeGPP1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, wp1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),Math.toRadians(180))
                .build();

        driveIntakeGPP1Intake = follower.pathBuilder()
                .addPath(new BezierLine(wp1Pose, gpp1Pose))
                .setLinearHeadingInterpolation(wp1Pose.getHeading(), gpp1Pose.getHeading())
                .build();

        driveIntakeGPP1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gpp1Pose, shootPose2))
                .setLinearHeadingInterpolation(gpp1Pose.getHeading(),shootPose.getHeading())
                .build();


        driveShootPosLeave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2,leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),leavePose.getHeading())
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
                    align();

                    action.Outtake_On(1);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 8) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Preload Shoot");

                    follower.followPath(driveShootPosIntakeGPP1, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP1);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKEGPP1:
                action.Stopper_On();

                if (!follower.isBusy()) {

                    panelsTelemetry.debug("Status", "Done shoot pos intakegpp1");

                    follower.followPath(driveIntakeGPP1Intake, 0.5,true);
                    setPathState(PathState.DRIVE_INTAKEGPP1INTAKE);
                }
                break;

            case DRIVE_INTAKEGPP1INTAKE:
                action.Intake_On(1);
                if (!follower.isBusy()) {
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Intake gpp 1 intake");

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
                    align();

                    action.Outtake_On(1);
                    action.Stopper_off();
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5) {
                    action.Intake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 8) {
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Shoot2");

                    follower.followPath(driveShootPosLeave, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_LEAVEPOS);
                }
                break;

            case DRIVE_SHOOTPOS_LEAVEPOS:
                action.Intake_Off();
                action.Outtake_Off();
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
        hood = new HoodControl(action,follower);

        limelight = new LimelightClass(hardwareMap);
        limelight.start();

        limelight.setTargetTagID(20);

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
        follower.setMaxPower(0.8);
        statePathUpdate();
        limelight.update();

        panelsTelemetry.debug("Path State", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Path Time", pathtimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("opMode Time", opModeTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }

    public void align(){
        if (limelight.hasTarget()) {

            double error = limelight.getTx();

            double derivative = error - lastError;
            lastError = error;

            double power = kP * error + kD * derivative;

            power = Math.max(-0.6, Math.min(0.6, power));

            action.Turret_SetPower(power);


            telemetry.addData("tx", error);
            telemetry.addData("Power", power);
        }
    }
}