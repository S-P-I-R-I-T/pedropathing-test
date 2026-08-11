package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.6)
            .forwardZeroPowerAcceleration(-36.05904039909434)
            .lateralZeroPowerAcceleration(-66.35943482955332)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05,0,0.012,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.002,0.025))
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants();

    public static MecanumConstants mecanumConstants = new MecanumConstants();

    static {
        mecanumConstants.leftFrontMotorName = "fl";
        mecanumConstants.leftRearMotorName = "rl";
        mecanumConstants.rightFrontMotorName = "fr";
        mecanumConstants.rightRearMotorName = "rr";
        mecanumConstants.xVelocity(57.24257652987512);
        mecanumConstants.yVelocity(45.4708718127153);

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.7)
            .strafePodX(2.5)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}