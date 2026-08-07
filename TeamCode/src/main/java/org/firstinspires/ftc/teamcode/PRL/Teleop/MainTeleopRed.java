package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.DriveClass;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.PRL.Class.TargetTracking;
import org.firstinspires.ftc.teamcode.PRL.Class.TurretClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "MainTeleopRed")
public class MainTeleopRed extends LinearOpMode {

    LimelightClass limelight;
    ActionManaging action;
    TurretClass turret;
    TargetTracking tracking;
    DriveClass drive;
    Follower follower;

    public static final int RED_TAG_ID = 24;

    public static final boolean UseTurretClass = true;

    public static final double START_X = 0;
    public static final double START_Y = 0;
    public static final double START_HEADING = 0;

    public static final double GOAL_X = 131;
    public static final double GOAL_Y = 135;

    @Override
    public void runOpMode() {

        limelight = new LimelightClass(hardwareMap);
        action = new ActionManaging(hardwareMap);
        drive = new DriveClass(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        if (PoseHolder.endPose != null) {
            follower.setStartingPose(PoseHolder.endPose);
        } else {
            follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        }

        if (UseTurretClass) {
            turret = new TurretClass(hardwareMap, limelight);
            turret.setGoal(GOAL_X, GOAL_Y);
        } else {
            tracking = new TargetTracking(limelight, action);
        }

        drive.init();

        limelight.setTargetTagID(RED_TAG_ID);
        limelight.start();

        waitForStart();

        action.Outtake_Off();

        while(opModeIsActive()){
            drive.drive(gamepad1);
            follower.update();

            telemetry.addData("Pose", "%.1f, %.1f, %.1f",
                    follower.getPose().getX(), follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));

            if (UseTurretClass) {
                turret.update(follower.getPose());
            } else {
                tracking.update(telemetry, gamepad2.right_bumper || gamepad2.left_bumper);
            }

            Intake();
            Outtake();

            telemetry.update();
        }

        limelight.stop();
    }

    void Intake(){
        if (gamepad2.a) {
            action.Intake_On();
            action.Stopper_On();
        } else if (gamepad2.b) {
            action.Intake_R();
            action.Stopper_On();
        } else {
            action.Intake_Off();
        }
    }

    void Outtake(){
        if (gamepad2.right_bumper){
            action.Outtake_On(1);
            if (action.Outtake_Velocity() > ActionManaging.Shooting_Far_Velocity){
                action.Intake_On();
            } else {
                action.Intake_Off();
            }
        } else if (gamepad2.left_bumper){
            action.Outtake_On(2);
            if (action.Outtake_Velocity() > ActionManaging.Shooting_Near_Velocity){
                action.Intake_On();
            } else {
                action.Intake_Off();
            }
        } else {
            action.Outtake_Off();
        }
    }
}
