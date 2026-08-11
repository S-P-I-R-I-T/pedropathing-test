package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class BLUETeleop extends LinearOpMode {

    LimelightClass limelight;
    ActionManaging action;
    Follower follower;
    public static boolean centric = true;
    public static final int BLUE_TAG_ID = 20;


    public static final double START_X = 0;
    public static final double START_Y = 0;
    public static final double START_HEADING = 0;

    boolean Is_Tracking = true;

    public static double kP = 0.02;
    public static double kD = 0.003;
    double lastError = 0;

    boolean lastRightStickButton = false;
    @Override
    public void runOpMode() {

        limelight = new LimelightClass(hardwareMap);
        action = new ActionManaging(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        if (PoseHolder.endPose != null) {
            follower.setStartingPose(PoseHolder.endPose);
        } else {
            follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        }


        limelight.setTargetTagID(BLUE_TAG_ID);
        limelight.start();

        waitForStart();
        follower.startTeleopDrive();

        while(opModeIsActive()){
            follower.update();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    centric);

            telemetry.addData("Velocity",action.Outtake_Velocity());

            LLtracking();

            Intake();
            Outtake();

            telemetry.update();
        }

        limelight.stop();
    }

    void Intake(){
        if (gamepad1.a) {

            if (gamepad2.right_bumper || gamepad2.left_bumper) {

                // Outtake와 동시에 사용 → 풀파워
                action.Intake_On(1);

            } else {

                // 일반 Intake → 느린 속도
                action.Intake_On(2);
            }

        } else {

            action.Intake_Off();
        }
    }

    void Outtake(){
        if (gamepad2.right_bumper){
            action.Outtake_On(1);
        } else if (gamepad2.left_bumper){
            action.Outtake_On(2);
        }

        if (gamepad2.dpad_down){
            action.Outtake_Off();
        }

        if (gamepad2.left_trigger_pressed){
            action.Outtake_Reverse();
        }
    }

    void LLtracking() {

        limelight.update();

        if (gamepad2.right_stick_button && !lastRightStickButton) {
            Is_Tracking = !Is_Tracking;
        }

        // D-pad 수동 조작 최우선
        if (gamepad2.dpad_left) {

            action.Turret_SetPower(0.3);

        } else if (gamepad2.dpad_right) {

            action.Turret_SetPower(-0.3);

        } else if (limelight.hasTarget() && Is_Tracking) {

            double error = limelight.getTx();

            double derivative = error - lastError;
            lastError = error;

            double power = kP * error + kD * derivative;

            power = Math.max(-0.6, Math.min(0.6, power));

            action.Turret_SetPower(power);

            telemetry.addData("tx", error);
            telemetry.addData("Power", power);

        } else {

            action.Turret_SetPower(0);
            telemetry.addLine("No Target");
        }

    }
}
