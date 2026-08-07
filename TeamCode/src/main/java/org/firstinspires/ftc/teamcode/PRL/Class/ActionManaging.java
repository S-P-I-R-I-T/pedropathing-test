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

    public static double Intake_Power = 1;
    public static double IntakeR_Power = 1;

    public static double Stopper_Open_Pos = 1;
    public static double Stopper_Close_Pos = 0.5;

    public static double Shooting_Far_Velocity = 2000;
    public static double Shooting_Near_Velocity = 1000;

    public static double Preheat_Velocity = 800;
    public static double Turret_S_f = 17;
    public static double Turret_S_p = 200;

    public static double Turret_MaxPower = 0.6;


    public static double Hood_Far = 0.6;
    public static double Hood_Near = 0.5;



    public ActionManaging(HardwareMap hardwareMap){
        this.hw = hardwareMap;

        Turret_R = hw.get(DcMotor.class,"Turret_R");
        Intake = hw.get(DcMotor.class, "Intake");
        Turret_S = hw.get(DcMotorEx.class, "Turret_S");

        Stopper = hw.get(Servo.class, "Stopper");
        Turret_H = hw.get(Servo.class, "Turret_H");

        Turret_R.setDirection(DcMotorSimple.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

        Turret_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Turret_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Turret_S.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(Turret_S_p,0,0,Turret_S_f);
        Turret_S.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void Intake_On(){
        Intake.setPower(Intake_Power);
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
            Hood_Set(Hood_Far);
            Turret_S.setVelocity(Shooting_Far_Velocity);
        } else if (zone == 2) {
            Hood_Set(Hood_Near);
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
        power = Math.max(-Turret_MaxPower, Math.min(Turret_MaxPower, power));
        Turret_R.setPower(power);
    }

    public void Turret_Stop(){
        Turret_R.setPower(0);
    }

    public void Turret_Lock(){
        Turret_R.setPower(0);
    }
}
