package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveClass {

    private final DcMotorEx fl, fr, rl, rr;
    private final IMU imu;

    public static double MaxSpeed = 1.0;
    public static double DpadRotateSpeed = 0.5;
    public static double SlowFactor = 0.8;
    public static double StickDeadzone = 0.1;

    public static RevHubOrientationOnRobot.LogoFacingDirection LogoFacing = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection UsbFacing = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public DriveClass(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        imu = hardwareMap.get(IMU.class, "imu");

        fl.setDirection(DcMotor.Direction.FORWARD);
        rl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LogoFacing, UsbFacing)));
        imu.resetYaw();
    }

    public void drive(Gamepad gamepad) {
        if (gamepad.left_stick_button) {
            imu.resetYaw();
        }

        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = 0;

        if (Math.abs(y) < StickDeadzone) y = 0;
        if (Math.abs(x) < StickDeadzone) x = 0;

        if (gamepad.dpad_left) {
            rx = -DpadRotateSpeed;
        } else if (gamepad.dpad_right) {
            rx = DpadRotateSpeed;
        } else {
            double stickMagnitude = Math.hypot(gamepad.right_stick_x, gamepad.right_stick_y);
            if (stickMagnitude > StickDeadzone) {
                rx = gamepad.right_stick_x * stickMagnitude;
            }
        }

        double trigger = Math.max(gamepad.left_trigger, gamepad.right_trigger);
        double speed = MaxSpeed * (1 - trigger * SlowFactor);

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        fl.setPower((rotY + rotX + rx) / denominator * speed);
        fr.setPower((rotY - rotX - rx) / denominator * speed);
        rl.setPower((rotY - rotX + rx) / denominator * speed);
        rr.setPower((rotY + rotX - rx) / denominator * speed);
    }
}
