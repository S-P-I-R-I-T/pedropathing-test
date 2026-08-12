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

@Autonomous(name = "BlueAuto")
public class BlueAuto extends OpMode {
    TelemetryManager panelsTelemetry;
    Follower follower;
    Timer pathtimer , opModeTimer;
    ActionManaging action;

    public enum PathState{
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKEGPP
    }

    PathState pathState;

    private final Pose startPose = new Pose(20,120,Math.toRadians(144));
    private final Pose shootPose = new Pose(41,99,Math.toRadians(144));

    private final Pose GPPPose = new Pose(19,82.5,Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosIntakeGPP;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPosIntakeGPP = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,new Pose(59,81),GPPPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), GPPPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 1){
                    action.Outtake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 3){
                    action.Intake_On(2);
                }
                if (!follower.isBusy() && pathtimer.getElapsedTimeSeconds() >= 5){
                    action.Intake_Off();
                    action.Outtake_Off();
                    panelsTelemetry.debug("Status", "Done Path1");

                    follower.followPath(driveShootPosIntakeGPP, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKEGPP);
                    break;
                }

                break;
            case DRIVE_SHOOTPOS_INTAKEGPP:
                action.Intake_On(1);
                if (!follower.isBusy()){
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Path2");
                    break;
                }

            default:
                panelsTelemetry.debug("Status", "No State Command");

                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathtimer.resetTimer();

    }

    @Override
    public void init(){
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

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
    @Override
    public void loop(){
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
