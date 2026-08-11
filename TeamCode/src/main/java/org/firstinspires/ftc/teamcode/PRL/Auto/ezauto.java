package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// 얼라이언스를 위한 패널티 유도 오토코드
@Autonomous
public class ezauto extends LinearOpMode {

    public static double BACKWARD_TIME = 1.0; // 뒤로 갈 시간 (초)
    public static double FORWARD_TIME = 1.0;  // 앞으로 갈 시간 (초)
    public static double MOVE_POWER = 0.5;    // 이동 파워 (0~1)

    DcMotor fl, rl, fr, rr;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        rl = hardwareMap.get(DcMotor.class, "rl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        rl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        timer.reset();
        boolean goingBackward = true;

        while (opModeIsActive()) {
            if (goingBackward) {
                // 뒤로 이동
                setPower(-MOVE_POWER);
                if (timer.seconds() >= BACKWARD_TIME) {
                    goingBackward = false;
                    timer.reset();
                }
            } else {
                // 앞으로 이동
                setPower(MOVE_POWER);
                if (timer.seconds() >= FORWARD_TIME) {
                    setPower(0);
                    break;
                }
            }

            telemetry.addData("Direction", goingBackward ? "Backward" : "Forward");
            telemetry.addData("Elapsed", "%.2f sec", timer.seconds());
            telemetry.update();
        }

        setPower(0);
    }

    private void setPower(double power) {
        fl.setPower(power);
        rl.setPower(power);
        fr.setPower(power);
        rr.setPower(power);
    }
}
