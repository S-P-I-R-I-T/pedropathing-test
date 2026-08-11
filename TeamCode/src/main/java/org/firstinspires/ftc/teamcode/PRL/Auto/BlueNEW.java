package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueNEW")
public class BlueNEW extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer , opModeTimer;

    public enum PathState{
        // Start Pos / End Pos
        // Drive > 움직임 상태
        // Shoot > 슈팅

        DRIVE_STARTPOS_SHOOTPOS ,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_GPPPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(20,120,Math.toRadians(144));
    private final Pose shootPose = new Pose(41,99,Math.toRadians(144));
    private final Pose GPPPose = new Pose(19,82.5,Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosGPPPos;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPosGPPPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,new Pose(59.000, 81.000),GPPPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),GPPPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);

                break;

            case SHOOT_PRELOAD:
                // Outtake on
                if (!follower.isBusy()){
                    // check is velocity high enough to shoot
                    // Intake on 1
                    telemetry.addLine("Done Path 1");

                    follower.followPath(driveShootPosGPPPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_GPPPOS);
                }
                break;

            case DRIVE_SHOOTPOS_GPPPOS:
                // Outtake Intake OFF!!
                if (!follower.isBusy()){
                    // check is velocity 0 or -
                    // Intake On 2

                    telemetry.addLine("Done Path 2");
                    //todo : pathState = NEXT;
                }
                break;

            default:
                telemetry.addLine("No State Command");
                break;
        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        // todo : add in any other mechanisms : actionmanaging

        buildPaths();

        follower.setPose(startPose);
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
        panelsTelemetry.debug("Path time", pathTimer.getElapsedTimeSeconds());

        panelsTelemetry.update(telemetry);
    }
}