package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * {@link Servo}를 그대로 구현한 쓰기-캐싱 래퍼.
 *
 * <p>SDK 문법 그대로 사용 가능: {@code claw.setPosition(0.6)} — 위치가 변했을 때만
 * 실제 전송. 집게처럼 "버튼 눌려있는 동안 계속 setPosition" 패턴에서
 * 매 루프 중복 PWM 명령을 제거한다.</p>
 *
 * <p>기본 데드밴드 0.001: 서보 PWM 분해능(약 0.004 스텝)보다 작아 실질 동작 차이 없음.</p>
 */
public class AServo implements Servo {

    private final Servo servo;
    private final double positionEpsilon;

    private double lastPosition = Double.NaN;   // NaN = 아직 안 보냄 → 첫 호출 무조건 전송
    private double lastMin = Double.NaN;
    private double lastMax = Double.NaN;
    private Direction lastDirection = null;     // Servo.Direction

    public AServo(Servo servo) {
        this(servo, 0.001);
    }

    /** @param positionEpsilon setPosition 변화 무시 임계값 (기본 0.001) */
    public AServo(Servo servo, double positionEpsilon) {
        this.servo = servo;
        this.positionEpsilon = positionEpsilon;
    }

    /** 원본 Servo 반환 (ServoImplEx 등 확장 기능 필요 시). */
    public Servo getRaw() {
        return servo;
    }

    /** 캐시를 전부 무효화한다. */
    public void resetWriteCache() {
        lastPosition = Double.NaN;
        lastMin = Double.NaN;
        lastMax = Double.NaN;
        lastDirection = null;
    }

    // ===================== 캐싱되는 쓰기 =====================

    @Override
    public void setPosition(double position) {
        if (Double.isNaN(lastPosition) || Math.abs(position - lastPosition) > positionEpsilon) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    @Override
    public void scaleRange(double min, double max) {
        if (min != lastMin || max != lastMax) {
            servo.scaleRange(min, max);
            lastMin = min;
            lastMax = max;
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction != lastDirection) {
            servo.setDirection(direction);
            lastDirection = direction;
        }
    }

    // ===================== 위임되는 읽기 =====================

    @Override public double getPosition()       { return servo.getPosition(); }
    @Override public Direction getDirection()   { return servo.getDirection(); }
    @Override public ServoController getController() { return servo.getController(); }
    @Override public int getPortNumber()        { return servo.getPortNumber(); }

    // ===================== HardwareDevice 위임 =====================

    @Override public Manufacturer getManufacturer()           { return servo.getManufacturer(); }
    @Override public String getDeviceName()                   { return servo.getDeviceName(); }
    @Override public String getConnectionInfo()               { return servo.getConnectionInfo(); }
    @Override public int getVersion()                         { return servo.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode() { servo.resetDeviceConfigurationForOpMode(); }
    @Override public void close()                             { servo.close(); }
}
