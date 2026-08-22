package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.function.Supplier;

/**
 * 비싼 읽기(I2C 센서 등)를 일정 주기마다만 수행하고, 나머지 루프에서는
 * 캐시된 값을 반환하는 제네릭 스로틀 래퍼.
 *
 * <p>원리: IMU/컬러/거리센서 같은 I2C 장치는 bulk read 대상이 아니어서
 * 읽기 1회당 3~7ms가 든다. 이 클래스는 N ms에 한 번만 실제로 읽고
 * 사이 구간은 마지막 값을 돌려준다.</p>
 *
 * <h2>사용법</h2>
 * <pre>{@code
 * private TimedSensor<Double> heading;
 *
 * @Override
 * public void init() {
 *     IMU imu = hardwareMap.get(IMU.class, "imu");
 *     heading = new TimedSensor<>(
 *         () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
 *         50);   // 50ms = 20Hz. 대부분의 용도에 충분
 * }
 *
 * @Override
 * public void loop() {
 *     double h = heading.read();   // 최근 50ms 내면 캐시 반환 (0ms)
 *     ...
 * }
 * }</pre>
 *
 * <p><b>Trade-off:</b> 값이 최대 intervalMs 만큼 늦게 갱신된다(여기선 최대 50ms).
 * 정밀한 고속 제어에는 주기를 줄일 것.</p>
 *
 * <p><b>⚠ 예외:</b> 오도메트리(Pinpoint 등)의 update()/getPosition()은 매 루프
 * 호출해야 한다 — Pedro Pathing 로컬라이저라서 스로틀하면 위치 추정이 망가진다.</p>
 *
 * @param <T> 센서 값 타입
 */
public class TimedSensor<T> {

    private final Supplier<T> reader;
    private final long intervalNanos;
    private final boolean skipNull;

    private T cached = null;
    private long lastReadNanos = Long.MIN_VALUE;

    /**
     * @param reader      실제 센서 읽기를 수행하는 람다
     * @param intervalMs  실제 읽기 사이 최소 간격 (ms)
     */
    public TimedSensor(Supplier<T> reader, long intervalMs) {
        this(reader, intervalMs, false);
    }

    /**
     * @param skipNull true이면 reader가 null을 반환했을 때 타이머를 갱신하지 않아
     *                 다음 read()에서 즉시 재시도한다 (Limelight result 등).
     */
    public TimedSensor(Supplier<T> reader, long intervalMs, boolean skipNull) {
        this.reader = reader;
        this.intervalNanos = intervalMs * 1_000_000L;
        this.skipNull = skipNull;
    }

    /** 매 루프 호출. 주기가 지났으면 실제로 읽고, 아니면 캐시를 반환한다. */
    public T read() {
        long now = System.nanoTime();
        if (lastReadNanos == Long.MIN_VALUE || now - lastReadNanos >= intervalNanos) {
            T value = reader.get();
            if (!(skipNull && value == null)) {
                cached = value;
                lastReadNanos = now;
            }
            return value != null ? value : cached;
        }
        return cached;
    }

    /** 타이머와 무관하게 즉시 강제 읽기. */
    public T refresh() {
        cached = reader.get();
        lastReadNanos = System.nanoTime();
        return cached;
    }

    /** 마지막으로 읽은 값 (실제 읽기 없이). 아직 안 읽었으면 null. */
    public T peekCached() {
        return cached;
    }

    /** {@link Func} 스타일 호환용 (telemetry.addData(String, Func) 등). */
    public Func<T> asFunc() {
        return this::read;
    }
}
