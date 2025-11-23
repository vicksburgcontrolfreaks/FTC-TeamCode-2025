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
    // Auton final pose constants
    public static double autonFinalX = 0.0;
    public static double autonFinalY = 0.0;
    public static double autonFinalHeading = 0.0;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-46.1394)
            .lateralZeroPowerAcceleration(-66.9937)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.06, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015, 0, 0.00001, 0.6, 0.01))
            .mass(11.5);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(56.4362105)
            .yVelocity(32.19775)
            .rightFrontMotorName("rf") //EH1
            .rightRearMotorName("rr") //EH0
            .leftRearMotorName("lr") //EH3
            .leftFrontMotorName("lf") //EH2
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.89) //175mm 6.89in
            .strafePodX(0) // 0
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //EH I2C Bus 0
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}



