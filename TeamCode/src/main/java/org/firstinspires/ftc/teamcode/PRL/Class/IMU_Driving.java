package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class IMU_Driving {
    public static class Vector2d {
        public double x;
        public double y;
        public Vector2d(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    public IMU_Driving(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.fl = hardwareMap.get(DcMotor.class,"fl");
        this.fr = hardwareMap.get(DcMotor.class,"fr");
        this.rl = hardwareMap.get(DcMotor.class,"rl");
        this.rr = hardwareMap.get(DcMotor.class,"rr");
        this.imu = hardwareMap.get(IMU.class,"imu");;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }
    public DcMotor fl,fr,rl,rr;
    public IMU imu;
    public Telemetry telemetry;
    public Gamepad gamepad1;

    public double speed = 1.0;
    double yaw;

    public void init(){
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        telemetry.addData("IMU: ", "INITIALIZED");
    }


    public double getYaw(){
        yaw = imu.getRobotYawPitchRollAngles().getYaw();

        telemetry.addData("yaw: ",yaw);
        return yaw;
    }

    public void resetYaw(StartPos pos){
        if(pos == StartPos.BLUE_L || pos == StartPos.RED_R){ //반시게방향 90도 회전
            rotate2Deg(90);
        }
        else if(pos == StartPos.BLUE_R || pos == StartPos.RED_L){// 시계방향 90도 회전
            rotate2Deg(-90);
        }
        else{
            return;
        }

        imu.resetYaw();
    }

    public double rotateSlowThreshold = 50;

    /**
     * GET YAW!!
     * @param targetYaw 타켓 각도
     * @return rx
     */
    public double getRotatePower(double targetYaw){
        //yaw 거리 전처리
        double yawDist = targetYaw - getYaw(); //GetYaw !!
        if(Math.abs(yawDist) > 180){
            yawDist -= Math.signum(yawDist) * 360;
        }
        double rx;
        if(Math.abs(yawDist) > rotateSlowThreshold){//거리가 임계값 이상 이면
            rx = Math.signum(yawDist); //rx = 최대(1)
        }
        else{
            rx = yawDist/ rotateSlowThreshold; // 거리가 임계값 이하면 거리에 반비례해 1~0
        }

        telemetry.addData("rotate: ", targetYaw + "/" + -rx + "/" + yawDist);
        return -rx;
    }

    /**
     *
     * @param targetYaw 타겟 각도
     * @see IMU_Driving#getRotatePower(double)
     */
    public boolean rotate2Deg(double targetYaw){
        double rx;
        do {
            rx = getRotatePower(targetYaw);
            fl.setPower(rx * speed);
            fr.setPower(-rx  * speed);
            rl.setPower(rx  * speed);
            rr.setPower(-rx  * speed);
        }while(Math.abs(getYaw()-targetYaw) > 4);
        return false;
    }

    public void controlWithPad(GamepadPurpose p){
        double rx = 0;
        double mX = 0;
        double mY = 0;

        //rotate: right stick
        if(p == GamepadPurpose.ROTATE || p == GamepadPurpose.WHOLE && !(Math.abs(gamepad1.right_stick_x) < 0.1 && Math.abs(gamepad1.right_stick_y) < 0.1)){
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;
            telemetry.addData("move: ", x + "/" + y );
            double targetYaw = -Math.toDegrees(Math.atan2(x,y)); // 90도 회전 (위 -> 0)
            if(Math.abs(targetYaw - yaw) > 1.5){
                rx = getRotatePower(targetYaw);
            }
        }

        //move: left stick
        if(p == GamepadPurpose.MOVE || p == GamepadPurpose.WHOLE){
            Vector2d moveVec = getMovePower();
            mX = moveVec.x;
            mY = moveVec.y;
        }

        //dpad
        if(gamepad1.dpad_left || gamepad1.dpad_right){
            rx = (gamepad1.dpad_right? 1:0) - (gamepad1.dpad_left? 1:0);
        }
        //init
        if((p == GamepadPurpose.MOVE || p == GamepadPurpose.WHOLE) && gamepad1.leftStickButtonWasPressed()){
            imu.resetYaw();
        }

        // 속도 조절 g1.rb -- / lb -
        speed = 0.7;
        if(gamepad1.right_bumper){
            speed = 0.3;
        }else if (gamepad1.left_bumper ){
            speed = 0.5;
        }



        double deno = JavaUtil.maxOfList(
                JavaUtil.createListWith(
                        Math.abs(mX),
                        Math.abs(mY),
                        Math.abs(rx),
                        1));
        fl.setPower((mX + mY +rx) / deno * speed);
        fr.setPower((-mX + mY -rx) / deno * speed);
        rl.setPower((-mX + mY +rx) / deno * speed);
        rr.setPower((mX + mY -rx) / deno * speed);
    }

    /**
     * GetYaw!!
     *
     * @return (dx, dy) - 이동 방향
     */
    Vector2d getMovePower(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        if(Math.abs(x) < 0.1 && Math.abs(y) < 0.1) return new Vector2d(0, 0);
        double radian = Math.toRadians(getYaw()); // 라디안 계산 때는 정방향 필요

        double a = x * Math.cos(radian) + y * Math.sin(radian);
        double b = x * -Math.sin(radian) + y * Math.cos(radian);

        if(Math.abs(a) < 0.00000025) a = 0;
        if(Math.abs(b) < 0.00000025) b = 0;
        telemetry.addData("move: ", a + "/" + b );
        return new Vector2d(a, b);
    }





    public enum StartPos{
        BLUE_L, //작은 삼각형(런치 존)
        BLUE_R, //큰 삼각형(런치 존)
        RED_L, //큰 삼각형(런치 존)
        RED_R; //작은 삼각형(런치 존)
    }
    public enum GamepadPurpose{
        MOVE,
        ROTATE,
        WHOLE;
    }

}