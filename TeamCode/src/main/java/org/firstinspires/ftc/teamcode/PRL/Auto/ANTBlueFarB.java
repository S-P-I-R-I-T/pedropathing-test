
package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable // Panels
public class ANTBlueFarB extends OpMode {
    TelemetryManager panelsTelemetry;
    Follower follower;
    Timer pathtimer, opModeTimer;
    ActionManaging action;
    HoodControl hood;

    public static double kP = 0.02;
    public static double kD = 0.003;
    double lastError = 0;
    LimelightClass limelight;

    public static double Leave_Time = 50;
    // 1분간 오토를 진행할때 공을 쏘고 전환시간이 된뒤 골이 된다면 패널티이므로 50초때 리브포인트로 이동함
    // 여기서 리브 포인트란 A에선 텔리옵 전환이 빠른 위치, B에선 선만 걸치는 파킹
    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INTAKE,
        INTAKE,
        DRIVE_INTAKE_SHOOTPOS,
        SHOOT,
        DRIVE_SHOOTPOS_LEAVEPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(55, 8, Math.toRadians(90));//todo: 오차 심하면 180으로 바꾸기!!
    private final Pose shootPose = new Pose(55, 15, Math.toRadians(180));

    private final Pose wpPose = new Pose(35, 15, Math.toRadians(180));
    private final Pose gppPose = new Pose(12, 15, Math.toRadians(180));

    private final Pose leavePose = new Pose(28, 20, Math.toRadians(180));
    // B에선 엔드게임을 위해서 파킹존에 걸치도록 함 todo: !! 현장 튜닝 필요 !! //


    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosIntake, driveIntakeGPP, driveGPPShootPos;
    private PathChain driveShootPosLeave;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, wpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), wpPose.getHeading())
                .build();

        driveIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(wpPose, gppPose))
                .setLinearHeadingInterpolation(wpPose.getHeading(), gppPose.getHeading())
                .build();

        driveGPPShootPos = follower.pathBuilder()
                .addPath(new BezierLine(gppPose, shootPose))
                .setLinearHeadingInterpolation(gppPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosLeave = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
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

                    follower.followPath(driveShootPosIntake, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INTAKE);
                }
                break;

            case DRIVE_SHOOTPOS_INTAKE:
                action.Stopper_On();

                if (!follower.isBusy()) {
                    panelsTelemetry.debug("Status", "Done shoot pos intake");

                    follower.followPath(driveIntakeGPP, 0.5, true);
                    setPathState(PathState.INTAKE);
                }
                break;

            case INTAKE:
                action.Intake_On(1);

                if (!follower.isBusy()) {
                    action.Intake_Off();
                    panelsTelemetry.debug("Status", "Done Intake");

                    follower.followPath(driveGPPShootPos, true);
                    setPathState(PathState.DRIVE_INTAKE_SHOOTPOS);
                }
                break;

            case DRIVE_INTAKE_SHOOTPOS:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
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
                    panelsTelemetry.debug("Status", "Done Shoot");

                    if (opModeTimer.getElapsedTimeSeconds() >= Leave_Time) {
                        follower.followPath(driveShootPosLeave, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_LEAVEPOS);
                    } else {
                        follower.followPath(driveShootPosIntake, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_INTAKE);
                    }
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
        hood = new HoodControl(action, follower);

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

    @Override
    public void stop() {
        limelight.stop();
    }

    public void align() {
        if (limelight.hasTarget()) {
            double error = limelight.getTx();

            double derivative = error - lastError;
            lastError = error;

            double power = kP * error + kD * derivative;

            power = Math.max(-0.7, Math.min(0.7, power));

            action.Turret_SetPower(power);

            telemetry.addData("tx", error);
            telemetry.addData("Power", power);
        }
    }
}
