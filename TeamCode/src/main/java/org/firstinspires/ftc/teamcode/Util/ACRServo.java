package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * {@link CRServo}를 그대로 구현한 쓰기-캐싱 래퍼.
 *
 * <p>SDK 문법 그대로 사용 가능: {@code intake.setPower(1.0)} — 값이 변했을 때만
 * 실제 전송. 인테이크처럼 매 루프 setPower를 호출하는 패턴에서 USB 트래픽 제거.</p>
 */
public class ACRServo implements CRServo {

    private final CRServo crServo;
    private final double powerEpsilon;

    private double lastPower = Double.NaN;      // NaN = 아직 안 보냄 → 첫 호출 무조건 전송
    private DcMotorSimple.Direction lastDirection = null;

    public ACRServo(CRServo crServo) {
        this(crServo, 0.005);
    }

    /** @param powerEpsilon setPower 변화 무시 임계값 (기본 0.005) */
    public ACRServo(CRServo crServo, double powerEpsilon) {
        this.crServo = crServo;
        this.powerEpsilon = powerEpsilon;
    }

    /** 원본 CRServo 반환. */
    public CRServo getRaw() {
        return crServo;
    }

    /** 캐시를 전부 무효화한다. */
    public void resetWriteCache() {
        lastPower = Double.NaN;
        lastDirection = null;
    }

    // ===================== 캐싱되는 쓰기 =====================

    @Override
    public void setPower(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > powerEpsilon) {
            crServo.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void setDirection(DcMotorSimple.Direction direction) {
        if (direction != lastDirection) {
            crServo.setDirection(direction);
            lastDirection = direction;
        }
    }

    // ===================== 위임되는 읽기 =====================

    @Override public double getPower()          { return crServo.getPower(); }
    @Override public DcMotorSimple.Direction getDirection() { return crServo.getDirection(); }
    @Override public ServoController getController() { return crServo.getController(); }
    @Override public int getPortNumber()        { return crServo.getPortNumber(); }

    // ===================== HardwareDevice 위임 =====================

    @Override public Manufacturer getManufacturer()           { return crServo.getManufacturer(); }
    @Override public String getDeviceName()                   { return crServo.getDeviceName(); }
    @Override public String getConnectionInfo()               { return crServo.getConnectionInfo(); }
    @Override public int getVersion()                         { return crServo.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { crServo.resetDeviceConfigurationForOpMode(); }
    @Override public void close()                             { crServo.close(); }
}
