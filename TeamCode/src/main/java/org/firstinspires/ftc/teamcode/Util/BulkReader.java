package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * 모든 Control/Expansion Hub를 MANUAL bulk caching 모드로 설정하고,
 * 루프당 한 번의 USB 통신으로 센서 데이터(엔코더/디지털/아날로그)를 몰아서 읽어오는 클래스.
 *
 * <p>원리: 기본 설정(OFF)에서는 {@code getCurrentPosition()} 같은 읽기 하나하나가
 * 개별 USB 트랜잭션(왕복 2~3ms)이라 루프가 느려진다. MANUAL 모드에서는
 * {@link #clear()} 호출 시 허브가 보유한 전체 센서 스냅샷을 1회에 가져오고,
 * 이후의 읽기는 로컬 캐시에서 즉시(0ms) 반환된다.</p>
 *
 * <h2>사용법</h2>
 * <pre>{@code
 * private BulkReader bulk;
 *
 * @Override
 * public void init() {
 *     bulk = new BulkReader(hardwareMap);   // init에서 1회
 * }
 *
 * @Override
 * public void loop() {
 *     bulk.clear();                         // ★ 반드시 loop() 맨 앞에서 1회!
 *     int pos = motor.getCurrentPosition(); // 캐시 히트 = 0ms
 *     ...
 * }
 * }</pre>
 *
 * <p><b>주의:</b> {@link #clear()}를 잊으면 이전 루프의 값(stale data)을 계속 읽게 된다.
 * loop() 첫 줄 습관화 필수.</p>
 */
public class BulkReader {

    private final List<LynxModule> hubs;

    /** init()에서 호출. 모든 허브를 MANUAL bulk caching 모드로 전환한다. */
    public BulkReader(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * 매 loop() 맨 앞에서 정확히 한 번 호출.
     * 모든 허브로부터 1회 bulk read를 수행해 센서 캐시를 갱신한다.
     */
    public void clear() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /** 관리 중인 허브 개수 (Control Hub + Expansion Hub). */
    public int hubCount() {
        return hubs.size();
    }
}
