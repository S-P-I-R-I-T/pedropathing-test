package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.IMU_Driving;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class REDTeleop extends LinearOpMode {

    LimelightClass limelight;
    ActionManaging action;
    public static final int RED_TAG_ID = 24;

    public static double Near_Hood = 0.35;
    public static double Far_Hood = 0.65;


    boolean Is_Tracking = true;

    public static double kP = 0.02;
    public static double kD = 0.003;
    public static double outtake_term = 0.2;
    double lastError = 0;

    boolean lastRightStickButton = false;

    boolean intakeToggle = false;
    double intakeToggleLastTime = 0;


    @Override
    public void runOpMode() {

        limelight = new LimelightClass(hardwareMap);
        action = new ActionManaging(hardwareMap);

        IMU_Driving imuDriving = new IMU_Driving(hardwareMap,telemetry,gamepad1);



        limelight.setTargetTagID(RED_TAG_ID);
        limelight.start();
        telemetry.update();

        waitForStart();

        imuDriving.init();
        imuDriving.getYaw();

        while(opModeIsActive()){
            imuDriving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);

            telemetry.addData("Velocity",action.Outtake_Velocity());


            LLtracking();

            Intake();
            Outtake();




            telemetry.update();
        }

        limelight.stop();
    }

    void Intake(){
        if (gamepad2.a ) {

            if (gamepad2.left_bumper) {

                // Outtake와 동시에 사용 → 풀파워
                action.Intake_On(2);


            } else if (gamepad2.right_bumper) {

                if (getRuntime() - intakeToggleLastTime > outtake_term) {
                    intakeToggle = !intakeToggle;
                    intakeToggleLastTime = getRuntime();
                }

                if (intakeToggle) {
                    action.Intake_On(2);
                } else {
                    action.Intake_Off();
                }
            } else {

                // 일반 Intake → 느린 속도
                action.Stopper_On();
                action.Intake_On(1);
            }

        } else if (gamepad2.b) {
            action.Intake_R();
        } else {

            action.Intake_Off();
            intakeToggle = false;
        }



    }

    void Outtake(){
        if (gamepad2.right_bumper){
            action.Stopper_off();
            action.Hood_Set(Far_Hood);

            action.Outtake_On(1);
        } else if (gamepad2.left_bumper){
            action.Stopper_off();
            action.Hood_Set(Near_Hood);

            action.Outtake_On(2);
        }

        if (gamepad2.dpad_down){
            action.Outtake_Off();
        }

        if (gamepad2.dpad_up){
            action.Outtake_On(3);
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
