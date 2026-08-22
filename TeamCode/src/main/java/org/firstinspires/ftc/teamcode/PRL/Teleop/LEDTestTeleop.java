package org.firstinspires.ftc.teamcode.PRL.Teleop;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Control/Expansion Hub 상태 LED 테스트용 TeleOp.
 *
 * <p>조작: A=빨강, B=파랑, X=초록, Y=흰색,
 * LB=초록 점멸, RB=빨강 점멸, D-pad Up=끄기,
 * D-pad Down(누르는 동안)=HSV 무지개 흐름</p>
 */
@TeleOp(name = "LEDTestTeleop", group = "PRL")
public class LEDTestTeleop extends LinearOpMode {

    private List<LynxModule> hubs;
    private int lastColor = Integer.MIN_VALUE;
    private boolean lastWasBlink = false;

    private static final long RAINBOW_SEND_INTERVAL_MS = 10;
    private static final float RAINBOW_PERIOD_MS = 1200f;
    private final ElapsedTime rainbowTimer = new ElapsedTime();
    private double lastRainbowSendMs = 0;

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);

        telemetry.addData("A/B/X/Y", "빨강/파랑/초록/흰색");
        telemetry.addData("LB/RB", "초록/빨강 점멸");
        telemetry.addData("D-pad Up", "LED 끄기");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                // HSV 색상환을 돌려 무지개가 연속으로 흐름.
                // 10ms 전송 간격은 USB 절약이 아니라 Lynx 명령 큐 포화 방지용 물리 하한.
                if (gamepad1.dpadDownWasPressed()) {
                    rainbowTimer.reset();
                    lastRainbowSendMs = -RAINBOW_SEND_INTERVAL_MS;
                }
                float t = (float) rainbowTimer.milliseconds();
                if (t - lastRainbowSendMs >= RAINBOW_SEND_INTERVAL_MS) {
                    lastRainbowSendMs = t;
                    float hue = t % RAINBOW_PERIOD_MS / RAINBOW_PERIOD_MS * 360f;
                    setConstant(Color.HSVToColor(new float[]{hue, 1f, 1f}));
                }
            }
            else if (gamepad1.aWasPressed())          setConstant(Color.RED);
            else if (gamepad1.bWasPressed())          setConstant(Color.rgb(0, 96, 255));
            else if (gamepad1.xWasPressed())          setConstant(Color.GREEN);
            else if (gamepad1.yWasPressed())          setConstant(Color.WHITE);
            else if (gamepad1.leftBumperWasPressed())  setBlink(Color.GREEN);
            else if (gamepad1.rightBumperWasPressed()) setBlink(Color.RED);
            else if (gamepad1.dpadUpWasPressed())      setConstant(Color.BLACK);

            String state = gamepad1.dpad_down ? "RAINBOW" : (lastWasBlink ? "BLINK" : "CONSTANT");
            telemetry.addData("State", state);
            telemetry.addData("Color", String.format("#%06X", lastColor & 0xFFFFFF));
            telemetry.update();
        }

        // 종료 후에도 마지막 색이 남으므로 검정으로 정리
        setConstant(Color.BLACK);
    }

    private void setConstant(int color) {
        if (!lastWasBlink && color == lastColor) {
            return;
        }
        lastColor = color;
        lastWasBlink = false;
        for (LynxModule hub : hubs) {
            hub.setConstant(color);
        }
    }

    private void setBlink(int color) {
        if (lastWasBlink && color == lastColor) {
            return;
        }
        lastColor = color;
        lastWasBlink = true;

        List<Blinker.Step> steps = new ArrayList<>();
        steps.add(new Blinker.Step(color, 150, TimeUnit.MILLISECONDS));
        steps.add(new Blinker.Step(Color.BLACK, 150, TimeUnit.MILLISECONDS));
        for (LynxModule hub : hubs) {
            hub.setPattern(steps);
        }
    }
}
