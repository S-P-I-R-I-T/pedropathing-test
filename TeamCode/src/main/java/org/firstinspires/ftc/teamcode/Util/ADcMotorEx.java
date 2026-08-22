package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * {@link DcMotorEx}를 그대로 구현(implements)한 쓰기-캐싱 래퍼.
 *
 * <p><b>SDK 문법 그대로 사용 가능:</b> DcMotorEx 자리에 이 클래스를 넣어도
 * {@code motor.setPower(0.5)}, {@code motor.setMode(...)} 등 호출 코드가 완전히 동일하다.
 * 차이는 내부적으로 값이 변했을 때만 실제 USB 전송을 한다는 것.</p>
 *
 * <ul>
 *   <li><b>쓰기(set*)</b>: 마지막 전송값과 비교해 변화가 있을 때만 USB 전송
 *       (setPower/setVelocity/setMode/setZeroPowerBehavior/setTargetPosition/
 *        setTargetPositionTolerance/setDirection/setMotorEnable|Disable)</li>
 *   <li><b>읽기(get*)</b>: 항상 원본 모터에 위임 → BulkReader(MANUAL) 모드에서
 *       엔코더/속도 읽기는 bulk cache 히트로 0ms</li>
 *   <li>PIDF 계수 설정, current alert 등 희귀 호출은 캐싱 없이 즉시 위임</li>
 * </ul>
 *
 * <h2>주의사항</h2>
 * <ul>
 *   <li>드라이브(메카넘) 모터는 Pedro Pathing follower에 <b>raw 인스턴스</b>를 넘길 것.
 *       이 래퍼는 슬라이드/집게/후드 등 <b>메커니즘 모터</b>용으로 권장.
 *       raw 인스턴스가 필요하면 {@link #getRaw()} 사용.</li>
 *   <li>{@link #getCurrent(CurrentUnit)}은 여전히 개별 USB 왕복(2~3ms).
 *       루프마다 호출하지 말 것 (필요 시 TimedSensor로 스로틀).</li>
 *   <li>raw 모터를 래퍼 밖에서 직접 조작하면 캐시가 어긋날 수 있으니
 *       {@link #resetWriteCache()}로 초기화할 것.</li>
 * </ul>
 */
public class ADcMotorEx implements DcMotorEx {

    private final DcMotorEx motor;
    private final double powerEpsilon;
    private final double velocityEpsilon;

    // ---- 캐시 상태 ----
    private double lastPower = Double.NaN;          // NaN = 아직 아무것도 안 보냄 → 첫 호출 무조건 전송
    private double lastVelValue = Double.NaN;
    private AngleUnit lastVelUnit = null;           // null = setVelocity(double) ticks 형태가 마지막
    private DcMotor.RunMode lastMode = null;
    private ZeroPowerBehavior lastZpb = null;
    private int lastTargetPosition = Integer.MIN_VALUE;
    private int lastTargetTolerance = Integer.MIN_VALUE;
    private Direction lastDirection = null;
    private Boolean lastMotorEnabled = null;        // null = unknown

    public ADcMotorEx(DcMotorEx motor) {
        this(motor, 0.005, 1.0);
    }

    /**
     * @param powerEpsilon   setPower 변화 무시 임계값 (기본 0.005)
     * @param velocityEpsilon setVelocity 변화 무시 임계값, ticks/s 또는 unit/s (기본 1.0)
     */
    public ADcMotorEx(DcMotorEx motor, double powerEpsilon, double velocityEpsilon) {
        this.motor = motor;
        this.powerEpsilon = powerEpsilon;
        this.velocityEpsilon = velocityEpsilon;
    }

    /** 원본 DcMotorEx 반환. Pedro follower에는 이 값을 넘긴다. */
    public DcMotorEx getRaw() {
        return motor;
    }

    /**
     * 캐시를 전부 무효화한다. raw 모터를 외부(Pedro 등)에서 건드린 뒤 호출하면
     * 다음 set* 호출이 확실히 하드웨어에 반영된다.
     */
    public void resetWriteCache() {
        lastPower = Double.NaN;
        lastVelValue = Double.NaN;
        lastVelUnit = null;
        lastMode = null;
        lastZpb = null;
        lastTargetPosition = Integer.MIN_VALUE;
        lastTargetTolerance = Integer.MIN_VALUE;
        lastDirection = null;
        lastMotorEnabled = null;
    }

    // ===================== 캐싱되는 쓰기 =====================

    @Override
    public void setPower(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > powerEpsilon) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    @Override
    public void setVelocity(double angularRate) {
        setVelocityCached(angularRate, null);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocityCached(angularRate, unit);
    }

    private void setVelocityCached(double rate, AngleUnit unit) {
        boolean changed = unit != lastVelUnit
                || Double.isNaN(lastVelValue)
                || Math.abs(rate - lastVelValue) > velocityEpsilon;
        if (changed) {
            if (unit == null) {
                motor.setVelocity(rate);
            } else {
                motor.setVelocity(rate, unit);
            }
            lastVelValue = rate;
            lastVelUnit = unit;
        }
    }

    @Override
    public void setMode(DcMotor.RunMode mode) {
        if (mode != lastMode) {
            motor.setMode(mode);
            lastMode = mode;
        }
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior != lastZpb) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
            lastZpb = zeroPowerBehavior;
        }
    }

    @Override
    public void setTargetPosition(int position) {
        if (position != lastTargetPosition) {
            motor.setTargetPosition(position);
            lastTargetPosition = position;
        }
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        if (tolerance != lastTargetTolerance) {
            motor.setTargetPositionTolerance(tolerance);
            lastTargetTolerance = tolerance;
        }
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction != lastDirection) {
            motor.setDirection(direction);
            lastDirection = direction;
        }
    }

    @Override
    public void setMotorEnable() {
        if (!Boolean.TRUE.equals(lastMotorEnabled)) {
            motor.setMotorEnable();
            lastMotorEnabled = Boolean.TRUE;
        }
    }

    @Override
    public void setMotorDisable() {
        if (!Boolean.FALSE.equals(lastMotorEnabled)) {
            motor.setMotorDisable();
            lastMotorEnabled = Boolean.FALSE;
        }
    }

    /** @deprecated raw 모터에 위임 후 내부 캐시(power=0, zpb=FLOAT)만 갱신 */
    @Deprecated
    @Override
    public void setPowerFloat() {
        motor.setPowerFloat();
        lastPower = 0.0;
        lastZpb = ZeroPowerBehavior.FLOAT;
    }

    // ===================== 위임되는 읽기 (bulk cache와 호환) =====================

    @Override public double getPower()                    { return motor.getPower(); }
    @Override public double getVelocity()                 { return motor.getVelocity(); }
    @Override public double getVelocity(AngleUnit unit)   { return motor.getVelocity(unit); }
    @Override public DcMotor.RunMode getMode()            { return motor.getMode(); }
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return motor.getZeroPowerBehavior(); }
    @Override public int getTargetPosition()              { return motor.getTargetPosition(); }
    @Override public int getTargetPositionTolerance()     { return motor.getTargetPositionTolerance(); }
    @Override public boolean isBusy()                     { return motor.isBusy(); }
    @Override public int getCurrentPosition()             { return motor.getCurrentPosition(); }
    @Override public boolean getPowerFloat()              { return motor.getPowerFloat(); }
    @Override public boolean isMotorEnabled()             { return motor.isMotorEnabled(); }
    @Override public Direction getDirection()             { return motor.getDirection(); }
    @Override public MotorConfigurationType getMotorType(){ return motor.getMotorType(); }
    @Override public DcMotorController getController()    { return motor.getController(); }
    @Override public int getPortNumber()                  { return motor.getPortNumber(); }

    // ===================== 희귀/무거운 호출: 즉시 위임 =====================
    // getCurrent(CurrentUnit)는 개별 USB 왕복(비쌈). 루프마다 호출 금지.

    @Override public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Deprecated
    @Override public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    @Deprecated
    @Override public PIDCoefficients getPIDCoefficients(DcMotor.RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override public void setMotorType(MotorConfigurationType motorType) { motor.setMotorType(motorType); }

    @Override public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override public double getCurrentAlert(CurrentUnit unit) { return motor.getCurrentAlert(unit); }

    @Override public double getCurrent(CurrentUnit unit) { return motor.getCurrent(unit); }

    @Override public boolean isOverCurrent() { return motor.isOverCurrent(); }

    // ===================== HardwareDevice 위임 =====================

    @Override public Manufacturer getManufacturer()          { return motor.getManufacturer(); }
    @Override public String getDeviceName()                  { return motor.getDeviceName(); }
    @Override public String getConnectionInfo()              { return motor.getConnectionInfo(); }
    @Override public int getVersion()                        { return motor.getVersion(); }
    @Override public void resetDeviceConfigurationForOpMode(){ motor.resetDeviceConfigurationForOpMode(); }
    @Override public void close()                            { motor.close(); }
}
