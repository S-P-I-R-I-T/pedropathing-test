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
import org.firstinspires.ftc.teamcode.Util.AGamepad;
import org.firstinspires.ftc.teamcode.Util.BulkReader;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class BLUETeleop extends LinearOpMode {

    LimelightClass limelight;
    ActionManaging action;
    Follower follower;
    private BulkReader bulk;
    public static final int BLUE_TAG_ID = 20;

    public static final double START_X = 9;
    public static final double START_Y = 9;
    public static final double START_HEADING = Math.toRadians(90);

    public static double Near_Hood = 0.35;
    public static double Far_Hood = 0.65;


    boolean Is_Tracking = true;

    public static double kP = 0.02;
    public static double kD = 0.003;
    public static double outtake_term = 0.2;
    double lastError = 0;

    boolean intakePulse = false;
    double intakePulseLastTime = 0;

    private AGamepad g2;


    @Override
    public void runOpMode() {

        bulk = new BulkReader(hardwareMap); // 모든 허브 MANUAL bulk 모드 전환
        g2 = new AGamepad(gamepad2);

        limelight = new LimelightClass(hardwareMap);
        action = new ActionManaging(hardwareMap);

        IMU_Driving imuDriving = new IMU_Driving(hardwareMap,telemetry,gamepad1);

        follower = Constants.createFollower(hardwareMap);
        if (PoseHolder.endPose != null) {
            follower.setStartingPose(PoseHolder.endPose);
        } else {
            follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING));
        }

        limelight.setTargetTagID(BLUE_TAG_ID);
        limelight.start();
        telemetry.update();

        // IMU 초기화/리셋은 INIT 단계에서 — start 후에 하면 첫 수백 ms 헤딩이 불안정
        imuDriving.init();
        imuDriving.getYaw();

        waitForStart();

        while(opModeIsActive()){
            bulk.clear(); // 루프 첫 줄 필수 — 빠지면 엔코더/센서 값 갱신 안 됨
            g2.update();  // 프레임 스냅샷 — 이후 g2.* 접근자는 몇 번 읽어도 동일 값

            follower.update();
            imuDriving.controlWithPad(IMU_Driving.GamepadPurpose.WHOLE);

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());

            telemetry.addData("Velocity", action.Outtake_Velocity());
            LLtracking();

            Intake();
            Outtake();

            telemetry.update();
        }

        limelight.stop();
    }

    void Intake(){
        if (g2.a.held()) {

            if (g2.lb.held()) {

                // Outtake와 동시에 사용 → 풀파워
                action.Intake_On(2);


            } else if (g2.rb.held()) {

                // A+RB 홀드: 0.2초 간격 온/오프 펄스 (엣지 토글 아님 — 동작 보존)
                if (getRuntime() - intakePulseLastTime > outtake_term) {
                    intakePulse = !intakePulse;
                    intakePulseLastTime = getRuntime();
                }

                if (intakePulse) {
                    action.Intake_On(2);
                } else {
                    action.Intake_Off();
                }
            } else {

                // 일반 Intake → 느린 속도
                action.Stopper_On();
                action.Intake_On(1);
            }

        } else if (g2.b.held()) {
            action.Intake_R();
        } else {

            action.Intake_Off();
            intakePulse = false;
        }



    }

    void Outtake(){
        if (g2.rb.held()){
            action.Stopper_off();
            action.Hood_Set(Far_Hood);

            action.Outtake_On(1);
        } else if (g2.lb.held()){
            action.Stopper_off();
            action.Hood_Set(Near_Hood);

            action.Outtake_On(2);
        }

        if (g2.dpadDown.held()){
            action.Outtake_Off();
        }

        if (g2.dpadUp.held()){
            action.Outtake_On(3);
        }
    }

    void LLtracking() {

        limelight.update();

        if (g2.rsBtn.pressed()) {
            Is_Tracking = !Is_Tracking;
        }

        // D-pad 수동 조작 최우선
        if (g2.dpadLeft.held()) {

            action.Turret_SetPower(0.3);

        } else if (g2.dpadRight.held()) {

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
