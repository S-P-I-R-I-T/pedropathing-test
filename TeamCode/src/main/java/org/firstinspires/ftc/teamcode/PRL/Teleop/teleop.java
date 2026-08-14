package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.ActionManaging;
import org.firstinspires.ftc.teamcode.PRL.Class.IMU_Driving;
import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;
import org.firstinspires.ftc.teamcode.PRL.Class.Mecanum_Driving;
import org.firstinspires.ftc.teamcode.PRL.Class.PoseHolder;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class teleop extends LinearOpMode {

    Follower follower;



    @Override
    public void runOpMode() {

        Mecanum_Driving md = new Mecanum_Driving(hardwareMap);
        telemetry.update();

        waitForStart();



        while(opModeIsActive()){
            double rt = gamepad1.right_trigger;

            double minSpeed = 0.3;
            double speed = 1.0 - (rt * (1.0 - minSpeed));

            md.setSpeed(speed);
            md.drive(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);


            telemetry.update();
        }

    }
}
