package org.firstinspires.ftc.teamcode.PRL.Class;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Util.ADcMotorEx;
import org.firstinspires.ftc.teamcode.Util.AServo;
import org.firstinspires.ftc.teamcode.Util.TimedSensor;

@Configurable
public class ActionManaging {
    // A-래퍼: 값이 변했을 때만 USB 전송. SDK 문법 동일(setPower/setPosition 등 그대로).
    ADcMotorEx Turret_R, Intake;
    ADcMotorEx Turret_S;
    AServo Stopper, Turret_H;
    HardwareMap hw;

    public static double Intake_Power = -0.7;
    public static double Intake_Power_OUTTAKE = -1;
    public static double IntakeR_Power = 1;

    public static double Stopper_Open_Pos = 1;
    public static double Stopper_Close_Pos = 0.63;

    public static double Shooting_Far_Velocity = 1850;
    public static double Shooting_Near_Velocity = 1100;

    public static double OuttakeReverse = - 1500;

    public static double Preheat_Velocity = 0;
    public static double Turret_S_f = 20;
    public static double Turret_S_p = 200;

    public static boolean Shooter_Volt_Comp = true;
    public static double Nominal_Battery_Voltage = 12.0;

    private final TimedSensor<Double> batteryVoltage;
    private static final long VOLTAGE_INTERVAL_MS = 200;
    private static final double MIN_VALID_VOLTAGE = 6.0;


    public ActionManaging(HardwareMap hardwareMap){
        this.hw = hardwareMap;

        // getVoltage는 bulk cache 밖의 개별 USB 트랜잭션(약 2ms)이라 200ms 스로틀로 읽음
        VoltageSensor vs = null;
        for (VoltageSensor s : hardwareMap.voltageSensor) { vs = s; break; }
        final VoltageSensor voltSensor = vs;
        batteryVoltage = new TimedSensor<>(
                () -> voltSensor != null ? voltSensor.getVoltage() : Nominal_Battery_Voltage,
                VOLTAGE_INTERVAL_MS);

        // raw 장치를 A-래퍼로 감싸기 (REV 모터는 전부 DcMotorEx를 구현하므로 안전)
        Turret_R = new ADcMotorEx(hw.get(DcMotorEx.class,"Turret_R"));
        Intake   = new ADcMotorEx(hw.get(DcMotorEx.class, "Intake"));
        Turret_S = new ADcMotorEx(hw.get(DcMotorEx.class, "Turret_S"));

        Stopper  = new AServo(hw.get(Servo.class, "Stopper"));
        Turret_H = new AServo(hw.get(Servo.class, "Turret_H"));

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

    /**
     * RUN_USING_ENCODER의 피드포워드(F·목표속도)는 배터리 전압을 가정하지 않아서,
     * 전압이 처지면 같은 목표 속도 도달이 느려진다. 명칭전압/실측전압 비율로 목표를 스케일해 보상.
     */
    private double compensatedVelocity(double ticksPerSecond){
        if (!Shooter_Volt_Comp) return ticksPerSecond;
        Double v = batteryVoltage.read();
        if (v == null || v < MIN_VALID_VOLTAGE) return ticksPerSecond;
        return ticksPerSecond * Nominal_Battery_Voltage / v;
    }

    public void Outtake_On(double zone){
        if (zone == 1) {
            Turret_S.setVelocity(compensatedVelocity(Shooting_Far_Velocity));
        } else if (zone == 2) {
            Turret_S.setVelocity(compensatedVelocity(Shooting_Near_Velocity));
        }else if (zone == 3){
            Turret_S.setVelocity(compensatedVelocity(OuttakeReverse));
        }
    }

    public void Outtake_Off(){
        Turret_S.setVelocity(Preheat_Velocity);
    }

    public double Outtake_Velocity(){
        return Turret_S.getVelocity();
    }

    public void Hood_Set(double Pos){
        Turret_H.setPosition(Pos);
    }

    public void Turret_SetPower(double power){
        Turret_R.setPower(power);
    }




}
