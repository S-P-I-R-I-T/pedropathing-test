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
    public static double Stopper_Close_Pos = 0.55;

    public static double Shooting_Far_Velocity = 2200;
    public static double Shooting_Near_Velocity = 1000;

    public static double Preheat_Velocity = 0;
    public static double Outtake_Reverse_Velocity = 0;
    public static double Turret_S_f = 20;
    public static double Turret_S_p = 200;

    public static double Hood_Far = 0.7;
    public static double Hood_Near = 0.3;



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

    public void Outtake_Reverse(){
        Turret_S.setVelocity(Outtake_Reverse_Velocity);
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

}
