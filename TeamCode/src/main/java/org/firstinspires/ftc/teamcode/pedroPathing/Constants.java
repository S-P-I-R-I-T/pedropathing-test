package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants();

    public static MecanumConstants mecanumConstants = new MecanumConstants();

    static {
        mecanumConstants.leftFrontMotorName = "fl";
        mecanumConstants.leftRearMotorName = "rl";
        mecanumConstants.rightFrontMotorName = "fr";
        mecanumConstants.rightRearMotorName = "rr";

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
