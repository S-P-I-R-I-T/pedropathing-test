package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum_Driving {

    DcMotor fl, fr, rl, rr;

    // 속도 계수 (0 ~ 1)
    double speedMultiplier = 1.0;

    public Mecanum_Driving(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // 외부에서 속도 조절
    public void setSpeed(double speed) {
        speedMultiplier = Math.max(0.0, Math.min(speed, 1.0));
    }

    public void drive(double y, double x, double rx) {

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double rlPower = y - x + rx;
        double rrPower = y + x - rx;

        double max = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(rlPower), Math.abs(rrPower))
        );

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            rlPower /= max;
            rrPower /= max;
        }

        // ⭐ 속도 계수 적용
        fl.setPower(flPower * speedMultiplier);
        fr.setPower(frPower * speedMultiplier);
        rl.setPower(rlPower * speedMultiplier);
        rr.setPower(rrPower * speedMultiplier);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}