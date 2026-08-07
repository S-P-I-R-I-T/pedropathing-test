package org.firstinspires.ftc.teamcode.PRL.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestAuto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {
            // Pre-run
            while (opModeIsActive()) {
                // OpMode loop

            }
        }
    }
}