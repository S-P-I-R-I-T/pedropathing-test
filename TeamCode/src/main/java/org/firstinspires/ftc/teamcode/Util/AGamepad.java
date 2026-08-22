package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * 게임패드 래퍼 — 프레임 스냅샷 방식.
 *
 * <h2>사용법</h2>
 * <pre>{@code
 * private final AGamepad driver = new AGamepad(gamepad1);
 * private final AGamepad op     = new AGamepad(gamepad2);
 *
 * while (opModeIsActive()) {
 *     driver.update();          // ★ 루프 첫 줄에서 1회 — 이후 접근자는 몇 번 읽어도 동일 값
 *     op.update();
 *
 *     if (driver.a.toggle())    intakeOn(); else intakeOff();
 *     if (op.rb.pressed())      shootOnce();
 *     if (op.rsBtn.released())  stopShooter();
 *     if (driver.rt.pressed(0.3)) boost();
 *     driver.rumbleBlips(2);    // 발사 준비 완료 진동
 * }
 * }</pre>
 *
 * <p>설계 원칙: {@link #update()}가 프레임당 한 번만 상태를 계산하고,
 * 접근자(pressed/toggle 등)는 부수효과 없는 순수 읽기라 호출 순서·횟수에 영향 없음.</p>
 */
public class AGamepad {

    public final Btn a, b, x, y;
    public final Btn dpadUp, dpadDown, dpadLeft, dpadRight;
    public final Btn lb, rb;
    public final Btn lsBtn, rsBtn;
    public final Btn startBtn, backBtn;

    public final Trigger lt, rt;

    private final Btn[] allButtons;
    private final Gamepad pad;

    public AGamepad(Gamepad gamepad) {
        pad = gamepad;

        a       = new Btn(() -> gamepad.a);
        b       = new Btn(() -> gamepad.b);
        x       = new Btn(() -> gamepad.x);
        y       = new Btn(() -> gamepad.y);

        dpadUp    = new Btn(() -> gamepad.dpad_up);
        dpadDown  = new Btn(() -> gamepad.dpad_down);
        dpadLeft  = new Btn(() -> gamepad.dpad_left);
        dpadRight = new Btn(() -> gamepad.dpad_right);

        lb = new Btn(() -> gamepad.left_bumper);
        rb = new Btn(() -> gamepad.right_bumper);

        lsBtn = new Btn(() -> gamepad.left_stick_button);
        rsBtn = new Btn(() -> gamepad.right_stick_button);

        startBtn = new Btn(() -> gamepad.start);
        backBtn  = new Btn(() -> gamepad.back);

        lt = new Trigger(() -> gamepad.left_trigger);
        rt = new Trigger(() -> gamepad.right_trigger);

        allButtons = new Btn[]{ a, b, x, y, dpadUp, dpadDown, dpadLeft, dpadRight,
                lb, rb, lsBtn, rsBtn, startBtn, backBtn };
    }

    /** 매 루프 첫 줄에서 정확히 1회 호출 — 전체 버튼/트리거 스냅샷 갱신 */
    public void update() {
        for (Btn btn : allButtons) {
            btn.update();
        }
        lt.update();
        rt.update();
    }

    // ---- 진동/LED 피드백 위임 ----

    public void rumble(int durationMs)              { pad.rumble(durationMs); }
    public void rumbleBlips(int count)              { pad.rumbleBlips(count); }
    public void stopRumble()                        { pad.stopRumble(); }
    public void setLedColor(int r, int g, int b, int durationMs) {
        pad.setLedColor(r, g, b, durationMs);
    }

    // ---- 스틱 유틸 ----

    /** 데드존 밖이면 원값, 안이면 0 (부호 보존) */
    public static double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) >= deadzone ? value : 0.0;
    }

    /**
     * 단일 버튼의 프레임 상태. update() 시점에 엣지·토글이 계산되고,
     * 접근자는 읽기 전용이라 루프 내 몇 번을 불러도 안전.
     */
    public class Btn {
        private final BooleanSupplier reader;
        private boolean prevHeld;
        private boolean toggleState;
        private boolean nowPressed;
        private boolean nowReleased;

        Btn(BooleanSupplier reader) {
            this.reader = reader;
        }

        void update() {
            boolean held = reader.getAsBoolean();
            nowPressed  =  held && !prevHeld;
            nowReleased = !held &&  prevHeld;
            if (nowPressed) {
                toggleState = !toggleState;
            }
            prevHeld = held;
        }

        /** 지금 눌려있는지 */
        public boolean held()      { return prevHeld; }
        /** 이 프레임에 누른 순간인지 (rising edge) */
        public boolean pressed()   { return nowPressed; }
        /** 이 프레임에 뗀 순간인지 (falling edge) */
        public boolean released()  { return nowReleased; }
        /** 누를 때마다 바뀌는 토글 상태 (true↔false) */
        public boolean toggle()    { return toggleState; }
        /** 토글 상태를 코드에서 강제 설정 */
        public void force(boolean state) { toggleState = state; }
    }

    /**
     * 트리거(아날로그). 임계값을 넘는 순간을 버튼처럼 쓸 수 있고,
     * 임계값은 읽을 때마다 달리 줘도 됨 (스냅샷은 update()에서 1회).
     */
    public class Trigger {
        private final DoubleSupplier reader;
        private double value;
        private double prevValue;

        Trigger(DoubleSupplier reader) {
            this.reader = reader;
        }

        void update() {
            prevValue = value;
            value = reader.getAsDouble();
        }

        /** 현재 아날로그 값 [0, 1] */
        public double value()                  { return value; }
        /** threshold를 넘은 순간 (rising edge) */
        public boolean pressed(double threshold)  { return value >= threshold && prevValue < threshold; }
        /** threshold 아래로 내려온 순간 (falling edge) */
        public boolean released(double threshold) { return value < threshold && prevValue >= threshold; }
        /** threshold 이상 유지 중인지 */
        public boolean held(double threshold)     { return value >= threshold; }
    }
}
