package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
            .mass(12.3)
            .forwardZeroPowerAcceleration(-29.5498)
            .lateralZeroPowerAcceleration(35.5886)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06,0,0,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7,0,0,0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.5,0.015))
            .centripetalScaling(0.0005)
            ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants();

    public static MecanumConstants mecanumConstants = new MecanumConstants();

    static {
        mecanumConstants.leftFrontMotorName = "fl";
        mecanumConstants.leftRearMotorName = "rl";
        mecanumConstants.rightFrontMotorName = "fr";
        mecanumConstants.rightRearMotorName = "rr";
        mecanumConstants.xVelocity(55.672290231299215);
        mecanumConstants.yVelocity(36.723804624061884);

        mecanumConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        mecanumConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        mecanumConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.6598)
            .strafePodX(-0.7426)
            .distanceUnit(DistanceUnit.INCH)
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