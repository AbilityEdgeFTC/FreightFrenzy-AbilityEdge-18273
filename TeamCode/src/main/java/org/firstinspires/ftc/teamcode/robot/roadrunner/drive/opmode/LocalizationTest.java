package org.firstinspires.ftc.teamcode.robot.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.T265Localizer;
//import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.DoubleLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
@Disabled
public class LocalizationTest extends LinearOpMode {

    public static double startPoseLeftX = -35;
    public static double startPoseLeftY = -60;
    public static double startPoseLeftH = 90;

    enum RobotLocalizer
    {
        Mecanum,
        Camera,
        Both
    }

    SampleMecanumDrive drive;
    T265Localizer t265Localizer;

    Pose2d poseEstimate;
    Trajectory trajectory;
    public static RobotLocalizer localizer = RobotLocalizer.Mecanum;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        switch (localizer)
        {
            case Mecanum:
                break;
            case Camera:
                t265Localizer = new T265Localizer(hardwareMap);
                drive.setLocalizer(t265Localizer);
                break;
            case Both:
                //drive.setLocalizer(new DoubleLocalizer(hardwareMap));
                break;
        }

        drive.setPoseEstimate(new Pose2d(startPoseLeftX, startPoseLeftY, startPoseLeftH));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        t265Localizer.stop();
    }
}
