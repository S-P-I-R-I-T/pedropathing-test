package org.firstinspires.ftc.teamcode.PRL.Class;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Util.ADcMotorEx;
import org.firstinspires.ftc.teamcode.Util.TimedSensor;

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
        this.fl = new ADcMotorEx(hardwareMap.get(DcMotorEx.class,"fl"));
        this.fr = new ADcMotorEx(hardwareMap.get(DcMotorEx.class,"fr"));
        this.rl = new ADcMotorEx(hardwareMap.get(DcMotorEx.class,"rl"));
        this.rr = new ADcMotorEx(hardwareMap.get(DcMotorEx.class,"rr"));
        this.imu = hardwareMap.get(IMU.class,"imu");
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.yawSensor = new TimedSensor<>(
                () -> imu.getRobotYawPitchRollAngles().getYaw(),
                YAW_INTERVAL_MS);
    }
    public ADcMotorEx fl,fr,rl,rr;
    public IMU imu;
    public Telemetry telemetry;
    public Gamepad gamepad1;

    private final TimedSensor<Double> yawSensor;
    private static final long YAW_INTERVAL_MS = 30;
    private static final double ROTATE_TIMEOUT_SECONDS = 3.0;

    public double speed = 1.0;
    double yaw;

    public void init(){
        // 좌측 REVERSE/우측 FORWARD 구성 — 기존엔 createFollower()가 이걸 대신 해줘서
        // follower 없이 실행하면 구동 방향이 틀어졌음 (루트 원인: 자체 구성 누락)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        ADcMotorEx[] drives = {fl, fr, rl, rr};
        for (ADcMotorEx m : drives) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        resetImuYaw();
        telemetry.addData("IMU: ", "INITIALIZED");
    }

    /** Pedro holdPoint/followPath 등이 raw 모터를 직접 쓴 뒤 호출 — 래퍼 캐시 동기화 */
    public void resetDriveWriteCache(){
        fl.resetWriteCache();
        fr.resetWriteCache();
        rl.resetWriteCache();
        rr.resetWriteCache();
    }

    private void resetImuYaw(){
        imu.resetYaw();
        yawSensor.refresh(); // 리셋 직후 값으로 캐시 갱신 (스로틀 stale 방지)
    }


    public double getYaw(){
        yaw = yawSensor.read();

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

        resetImuYaw();
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

        telemetry.addData("rotate", "%.1f/%.3f/%.1f", targetYaw, -rx, yawDist);
        return -rx;
    }

    /**
     * 목표 각도까지 제자리 회전 (blocking).
     * 탈출 조건: 4° 도달 / 3초 타임아웃 / DS STOP으로 스레드 interrupt.
     * @return 목표각 도달 여부
     */
    public boolean rotate2Deg(double targetYaw){
        final double tolerance = 4;
        ElapsedTime timeout = new ElapsedTime();
        try {
            while (!Thread.currentThread().isInterrupted()
                    && timeout.seconds() < ROTATE_TIMEOUT_SECONDS
                    && Math.abs(getYaw() - targetYaw) > tolerance) {
                double rx = getRotatePower(targetYaw);
                fl.setPower(rx * speed);
                fr.setPower(-rx * speed);
                rl.setPower(rx * speed);
                rr.setPower(-rx * speed);
                Thread.sleep(5); // 풀스핀 방지 — I2C/시스템 숨 쉴 틈
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // STOP 요청 → 플래그 복원 후 즉시 탈출
        }
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
        return Math.abs(getYaw() - targetYaw) <= tolerance;
    }

    public void controlWithPad(GamepadPurpose p){
        double rx = 0;
        double mX = 0;
        double mY = 0;

        //rotate: right stick — 괄호 명시: ROTATE 모드는 deadzone 무시, WHOLE은 스틱 입력 시에만
        if(p == GamepadPurpose.ROTATE
                || (p == GamepadPurpose.WHOLE && !(Math.abs(gamepad1.right_stick_x) < 0.1 && Math.abs(gamepad1.right_stick_y) < 0.1))){
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;
            telemetry.addData("aim", "%.2f/%.2f", x, y);
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
            resetImuYaw();
        }

        // 속도 조절 g1.rb -- / lb -
        speed = 0.7;
        if(gamepad1.right_bumper){
            speed = 0.3;
        }else if (gamepad1.left_bumper ){
            speed = 0.5;
        }



        // 바퀴 파워를 먼저 계산한 뒤 실제 최댓값으로 정규화 — 성분 기반 deno는 대각선 입력 시
        // 일부 바퀴만 ±1 클램프돼서 주행 방향이 휘어지는 버그였음
        double flP =  mX + mY + rx;
        double frP = -mX + mY - rx;
        double rlP = -mX + mY + rx;
        double rrP =  mX + mY - rx;

        double maxAbs = Math.max(Math.abs(flP), Math.max(Math.abs(frP), Math.max(Math.abs(rlP), Math.abs(rrP))));
        if (maxAbs > 1) {
            flP /= maxAbs;
            frP /= maxAbs;
            rlP /= maxAbs;
            rrP /= maxAbs;
        }

        fl.setPower(flP * speed);
        fr.setPower(frP * speed);
        rl.setPower(rlP * speed);
        rr.setPower(rrP * speed);
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
        telemetry.addData("move", "%.3f/%.3f", a, b);
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