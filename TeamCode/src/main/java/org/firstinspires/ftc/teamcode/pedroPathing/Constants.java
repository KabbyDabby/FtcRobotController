package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-182.18)
            .lateralZeroPowerAcceleration(-182.18)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0 , 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(.75, 0 , 0, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.1, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0035, 0, 0.00001, 0.6, 0.13))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.000005, 0.6, 0.13))
            .mass(5.362); //TODO: actually weigh the robot, in kg

    //top left = 1191g
    //top right = 1400g
    //bottom left = 1376g
    //bottom right = 1395g
    //total = 5362g

    public static SwerveConstants driveConstants = new SwerveConstants()
            .velocity(77.45)
//            .maxPower(.75)
            .useBrakeModeInTeleOp(true)
            .leftFrontServoName("ss2")
            .leftFrontEncoderName("se2")
            .leftFrontMotorName("sm2")
            .rightFrontServoName("ss1")
            .rightFrontEncoderName("se1")
            .rightFrontMotorName("sm1")
            .leftRearServoName("ss3")
            .leftRearEncoderName("se3")
            .leftRearMotorName("sm3")
            .rightRearServoName("ss0")
            .rightRearEncoderName("se0")
            .rightRearMotorName("sm0")
//            .leftFrontTurnPID(new PIDFCoefficients(0.003, 0.0, 0.001, 0.0))
//            .rightFrontTurnPID(new PIDFCoefficients(0.003, 0.0, 0.001, 0.0))
//            .leftRearTurnPID(new PIDFCoefficients(0.003, 0.0, 0.001, 0.0))
//            .rightRearTurnPID(new PIDFCoefficients(0.003, 0.0, 0.001, 0.0))
            .leftFrontTurnPID(new PIDFCoefficients(0.003,0, 0.000, 0.0))
            .rightFrontTurnPID(new PIDFCoefficients(0.003, 0.0, 0.000, 0.0))
            .leftRearTurnPID(new PIDFCoefficients(0.003, 0.0, 0.000, 0.0))
            .rightRearTurnPID(new PIDFCoefficients(0.003, 0.0, 0.000, 0.0))
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontServoDirection(CRServo.Direction.FORWARD)
            .rightFrontServoDirection(CRServo.Direction.FORWARD)
            .leftRearServoDirection(CRServo.Direction.FORWARD)
            .rightRearServoDirection(CRServo.Direction.FORWARD)
            .leftFrontPodAngleOffsetDeg(-81.38)
            .rightFrontPodAngleOffsetDeg(-11.0182)
            .leftRearPodAngleOffsetDeg(34.16)
            .rightRearPodAngleOffsetDeg(-71.6727)
            .leftFrontPodXYOffsets(new double[] { -174.5, 143.8745 })
            .rightFrontPodXYOffsets(new double[] { 174.5, 143.8745 })
            .leftRearPodXYOffsets(new double[] { -174.5, -143.8745 })
            .rightRearPodXYOffsets(new double[] { 174.5, -143.8745 });

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.3622) //60mm
            .strafePodX(-0.0001968) //-0.005mm
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
