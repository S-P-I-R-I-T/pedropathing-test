package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.AutoAlign;
import org.firstinspires.ftc.teamcode.PRL.Class.HoodControl;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class BLUETeleop extends LinearOpMode {

    LimelightClass limelight;
    ActionManaging action;
    Follower follower;
    HoodControl hood;
    AutoAlign align;
    public static boolean centric = true;
    public static final int BLUE_TAG_ID = 20;


    public static final double START_X = 20;
    public static final double START_Y = 120;
    public static final double START_HEADING = Math.toRadians(144);

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

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

        hood = new HoodControl(action,follower);
        align = new AutoAlign(action,follower);


        limelight.setTargetTagID(BLUE_TAG_ID);
        limelight.start();

        telemetry.addData("Turret", "터렛을 정면으로 맞추고 gamepad2 B를 누르세요");
        telemetry.update();
        while (!gamepad2.b && !isStopRequested()) {
            idle();
        }
        action.Turret_ResetZero();
        telemetry.addData("Turret", "Zero OK. Angle: %.1f", action.Turret_CurrentAngle());
        telemetry.update();

        waitForStart();
        follower.startTeleopDrive();

        while(opModeIsActive()){
            follower.update();
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    centric // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    centric // Robot Centric
            );

            if (gamepad1.rightBumperWasPressed()) {
                slowMode = !slowMode;
            }
            if (gamepad1.xWasPressed()) {
                slowModeMultiplier += 0.25;
            }
            //Optional way to change slow mode strength
            if (gamepad1.yWasPressed()) {
                slowModeMultiplier -= 0.25;
            }
            telemetry.addData("SlowMode",slowModeMultiplier);
            telemetry.addData("Velocity",action.Outtake_Velocity());
            telemetry.addData("TurretPos", action.Turret_Position());
            telemetry.addData("TurretAngle", action.Turret_CurrentAngle());

            LLtracking();

            Intake();
            Outtake();

            hood.update();

            telemetry.update();
        }

        limelight.stop();
    }

    void Intake(){
        if (gamepad1.a) {

            if (gamepad2.right_bumper || gamepad2.left_bumper) {

                // Outtake와 동시에 사용 → 풀파워
                action.Intake_On(2);

            } else {

                // 일반 Intake → 느린 속도
                action.Stopper_On();
                action.Intake_On(1);
            }

        } else {

            action.Intake_Off();
        }
    }

    void Outtake(){
        if (gamepad2.right_bumper){
            action.Stopper_off();

            action.Outtake_On(1);
        } else if (gamepad2.left_bumper){
            action.Stopper_off();

            action.Outtake_On(2);
        }

        if (gamepad2.dpad_down){
            action.Outtake_Off();
        }
    }

    void LLtracking() {

        limelight.update();

        boolean leftTriggerHeld = gamepad2.left_trigger > 0.5;
        boolean rightTriggerHeld = gamepad2.right_trigger > 0.5;

        // D-pad 수동 조작 최우선
        if (gamepad2.dpad_left) {

            action.Turret_SetAngle(action.Turret_CurrentAngle() + 2);

        } else if (gamepad2.dpad_right) {

            action.Turret_SetAngle(action.Turret_CurrentAngle() - 2);

        } else if (rightTriggerHeld) {

            // 오토얼라인 (포즈 기반)
            action.Turret_PowerMode();
            align.autoAlign(true); // BLUE

        } else if (leftTriggerHeld) {

            // 라임라이트 트래킹
            if (limelight.hasTarget()) {
                action.Turret_SetAngle(action.Turret_CurrentAngle() + limelight.getTx());
                telemetry.addData("tx", limelight.getTx());
            } else {
                action.Turret_SetAngle(action.Turret_CurrentAngle());
                telemetry.addLine("No Target");
            }

        } else {

            action.Turret_SetAngle(action.Turret_CurrentAngle());
        }

        telemetry.addData("TurretAngle", action.Turret_CurrentAngle());
    }
}
