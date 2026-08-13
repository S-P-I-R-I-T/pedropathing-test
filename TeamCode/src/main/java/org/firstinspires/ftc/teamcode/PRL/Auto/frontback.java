package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "frontback")
public class frontback extends LinearOpMode {

    Servo servo;
    DcMotor fl, fr, rl, rr;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        waitForStart();

        if (opModeIsActive()) {
            timer.reset();

            while (opModeIsActive() && timer.seconds() < 2.0) {
                front();
            }
            stopMotors();

            sleep(500);

            timer.reset();

            while (opModeIsActive() && timer.seconds() < 2.0) {
                back();
            }
            stopMotors();
        }
    }

    void front() {
        fl.setPower(-1);
        fr.setPower(1);
        rl.setPower(1);
        rr.setPower(-1);
    }
    void back() {
        fl.setPower(1);
        fr.setPower(-1);
        rl.setPower(1);
        rr.setPower(-1);
    }
    void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}