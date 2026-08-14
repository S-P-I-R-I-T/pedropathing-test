package org.firstinspires.ftc.teamcode.PRL.Class;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class ActionManaging {
    DcMotor Turret_R, Intake;
    DcMotorEx Turret_S;
    Servo Stopper, Turret_H;
    HardwareMap hw;

    public static double Intake_Power = -0.7;
    public static double Intake_Power_OUTTAKE = -1;
    public static double IntakeR_Power = 1;

    public static double Stopper_Open_Pos = 1;
    public static double Stopper_Close_Pos = 0.5;

    public static double Shooting_Far_Velocity = 1850;
    public static double Shooting_Near_Velocity = 1100;

    public static double Preheat_Velocity = 0;
    public static double Turret_S_f = 20;
    public static double Turret_S_p = 200;

    public static double Hood_Far = 0.7;
    public static double Hood_Near = 0.3;

    public static double Turret_TicksPerRev = 0.8; // 터렛 1회전(360도)당 인코더 틱. 28 × 모터기어비 × 터렛감속비. 직접 측정해 설정
    public static double Turret_MaxPower = 0.5;



    public ActionManaging(HardwareMap hardwareMap){
        this.hw = hardwareMap;

        Turret_R = hw.get(DcMotor.class,"Turret_R");
        Intake = hw.get(DcMotor.class, "Intake");
        Turret_S = hw.get(DcMotorEx.class, "Turret_S");

        Stopper = hw.get(Servo.class, "Stopper");
        Turret_H = hw.get(Servo.class, "Turret_H");

        Turret_R.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Turret_S.setDirection(DcMotorSimple.Direction.REVERSE);

        Turret_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret_S.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Turret_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Turret_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Turret_S_p,0,0,Turret_S_f);
        Turret_S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void Intake_On(double mode){
        if (mode == 1) {
            Intake.setPower(Intake_Power);
        } else if (mode ==2) {
            Intake.setPower(Intake_Power_OUTTAKE);
        }
    }
    public void Intake_R(){
        Intake.setPower(IntakeR_Power);
    }
    public void Intake_Off(){
        Intake.setPower(0);
    }

    public void Stopper_On(){
        Stopper.setPosition(Stopper_Close_Pos);
    }

    public void Stopper_off(){
        Stopper.setPosition(Stopper_Open_Pos);
    }

    public void Outtake_On(double zone){
        if (zone == 1) {
            Turret_S.setVelocity(Shooting_Far_Velocity);
        } else if (zone == 2) {
            Turret_S.setVelocity(Shooting_Near_Velocity);
        }
    }

    public void Outtake_Off(){
        Turret_S.setVelocity(Preheat_Velocity);
    }

    public double Outtake_Velocity(){
        return Turret_S.getVelocity();
    }

    public int Turret_Position(){
        return Turret_R.getCurrentPosition();
    }

    public void Hood_Set(double Pos){
        Turret_H.setPosition(Pos);
    }

    public void Turret_SetPower(double power){
        Turret_R.setPower(power);
    }

    // 파워 직접 제어 모드로 전환 (autoAlign 등에서 사용). 이후 Turret_SetAngle을 호출하면 다시 위치 모드.
    public void Turret_PowerMode(){
        Turret_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // 운영자가 터렛을 정면(로봇 진행 방향)으로 맞춘 뒤 호출하면, 이후 인코더 0 = 정면.
    public void Turret_ResetZero(){
        Turret_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double Turret_CurrentAngle(){
        return Turret_R.getCurrentPosition() / Turret_TicksPerRev * 360.0;
    }

    // 목표 각도(정면 0도, 우측 +, 좌측 -)로 위치 제어. 연속 회전이 가능하므로
    // -180~180 밖의 각도도 들어오면 그대로 목표로 이동.
    public void Turret_SetAngle(double angleDeg){
        int target = (int)Math.round(angleDeg / 360.0 * Turret_TicksPerRev);
        Turret_R.setTargetPosition(target);
        Turret_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret_R.setPower(Turret_MaxPower);
    }

}
