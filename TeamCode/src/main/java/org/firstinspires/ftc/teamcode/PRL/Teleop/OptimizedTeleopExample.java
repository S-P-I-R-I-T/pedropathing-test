package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.ADcMotorEx;
import org.firstinspires.ftc.teamcode.Util.AServo;
import org.firstinspires.ftc.teamcode.Util.BulkReader;
import org.firstinspires.ftc.teamcode.Util.TimedSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Util 최적화 클래스들의 실제 적용 예시.
 *
 * <p>적용 순서 요약:</p>
 * <ol>
 *   <li>init(): BulkReader 생성 + 메커니즘 하드웨어를 Caching* 래퍼로 감싸기</li>
 *   <li>loop() 맨 앞: bulk.clear() 1회</li>
 *   <li>loop(): 읽기 → 계산 → 쓰기 → 텔레메트리(스로틀) 순서 유지</li>
 *   <li>루프 타임 측정해 개선 확인 (목표 &lt;15ms)</li>
 * </ol>
 *
 * <p><b>사전 준비:</b> Robot Config에 "slide"(모터), "claw"(서보), "imu" 가 있어야 한다.
 * 이름은 실제 로봇 구성에 맞게 수정할 것.</p>
 *
 * <p><b>Pedro Pathing 주의:</b> 드라이브 모터(fl/rl/fr/rr)는 그대로 raw DcMotorEx로
 * {@code Constants.createFollower(hardwareMap)}에 넘긴다. 래퍼는 메커니즘 전용.</p>
 */
@TeleOp(name = "OptimizedTeleopExample", group = "PRL")
@Disabled   // 하드웨어 이름 맞춘 뒤 이 줄을 지우면 DS 목록에 나타남
public class OptimizedTeleopExample extends OpMode {

    // ---- 최적화 유틸 ----
    private BulkReader bulk;                       // 허브 bulk read 관리
    private ADcMotorEx slide;                      // 메커니즘 모터 (쓰기 캐싱)
    private AServo claw;                           // 서보 (쓰기 캐싱)
    private TimedSensor<Double> heading;           // IMU 스로틀 읽기 (20Hz)

    private int loopCount = 0;

    @Override
    public void init() {
        // ===== PHASE 0: bulk read 설정 — 반드시 다른 하드웨어 참조 전에 해도 무방 =====
        bulk = new BulkReader(hardwareMap);

        // ===== 메커니즘: 래퍼로 감싸기 (문법은 SDK와 동일) =====
        slide = new ADcMotorEx(hardwareMap.get(DcMotorEx.class, "slide"));
        claw  = new AServo(hardwareMap.get(Servo.class, "claw"));

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== I2C 센서: TimedSensor로 스로틀 (IMU 예시) =====
        IMU imu = hardwareMap.get(IMU.class, "imu");
        heading = new TimedSensor<>(
                () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                50);    // 50ms = 20Hz. Pinpoint 헤딩을 쓴다면 이 블록 자체가 불필요

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopCount++;

        // ================================================================
        // PHASE 1: bulk cache 갱신 — ★ 반드시 루프 맨 앞에서 딱 1회 ★
        // ================================================================
        bulk.clear();

        // ================================================================
        // PHASE 2: 읽기 (엔코더 = bulk cache 히트 0ms, IMU = 스로틀됨)
        // ================================================================
        int slidePos = slide.getCurrentPosition();     // 캐시 히트 = 공짜
        double yaw = heading.read();                   // 50ms마다만 실제 I2C

        // ================================================================
        // PHASE 3: 계산 (CPU만 — 비용 사실상 0)
        // ================================================================
        double targetPower = -gamepad1.left_stick_y;   // 왼쪽 스틱으로 슬라이드 제어 예시

        // ================================================================
        // PHASE 4: 쓰기 (래퍼가 값 변화 시에만 USB 전송)
        // ================================================================
        slide.setPower(targetPower);

        if (gamepad1.a)      claw.setPosition(0.6);    // 열기
        else if (gamepad1.b) claw.setPosition(0.0);    // 닫기

        // ================================================================
        // PHASE 5: 텔레메트리 — 10루프마다 1회 (WiFi 비용 절감)
        // ================================================================
        if (loopCount % 10 == 0) {
            telemetry.addData("Loop #", loopCount);
            telemetry.addData("Slide pos", slidePos);
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(yaw));
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        // 안전: 정지 시 메커니즘 확실히 멈추기 (래퍼 내려도 값이 같으면 무시되므로
        // 강제 전송이 필요하면 getRaw().setPower(0) 사용 가능)
        slide.setPower(0);
    }
}
