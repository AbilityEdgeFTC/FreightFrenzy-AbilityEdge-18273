package org.firstinspires.ftc.teamcode.opmodes.autonmous.t265;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Left BLUE T265", group = "blue")
@Disabled
public class AutoLeftBlue265 extends LinearOpMode {

    public static double startPoseRightX = 9.6;
    public static double startPoseRightY = 64.04;
    public static double startPoseRightH = 180;
    public static double poseHubFrontX = -11;
    public static double poseHubFrontY = 41.5;
    public static double poseHubFrontH = 270;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = 63.5;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 60;
    public static double poseCollectY = 63.5;
    public static double poseCollectH = 180;
    carousel carousel;
    intake intake;
    dip dip;
    //Elevator elevator;
    //ElevatorThreadAuto threadAuto;
    public static double reverseIntakeFor = .8;
    OpenCvWebcam webcam;
    public static boolean withVision = true;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;
    HSVPipeline pipeline;
    SampleMecanumDrive drive;

    Pose2d startPoseRight, poseHubFront, poseEntrance, poseCollect;
    TrajectorySequence placement, entrance, collect, cycle, entrance2, collect2;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        //threadAuto = new ElevatorThreadAuto(hardwareMap);
        //elevator = new Elevator(hardwareMap);

        drive.setPoseEstimate(startPoseRight);

        initPipeline();
        webcam.setPipeline(pipeline);

        placement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHubFront)
                .build();

        entrance = drive.trajectorySequenceBuilder(placement.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        collect = drive.trajectorySequenceBuilder(entrance.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-20,poseCollect.getY(),poseCollect.getHeading() - Math.toRadians(1.5)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-15,poseCollect.getY(),poseCollect.getHeading() + Math.toRadians(1.5)))
                .build();

        cycle = drive.trajectorySequenceBuilder(collect.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        entrance2 = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        collect2 = drive.trajectorySequenceBuilder(entrance2.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 10, poseCollect.getY(), poseCollect.getHeading() - Math.toRadians(2)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 7.5, poseCollect.getY(), poseCollect.getHeading() + Math.toRadians(3)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 5, poseCollect.getY(), poseCollect.getHeading() + Math.toRadians(3)))
                .build();


        dip.getFreight();
        //threadAuto.start();

        while (!opModeIsActive() && !isStopRequested())
        {
            switch (pipeline.getLocation()) {
                case Left:
                    //IF BARCODE IS ON LEFT SIDE
                    placeFreightIn = levels.MAX;
                    break;
                case Center:
                    //IF BARCODE IS ON CENTER SIDE
                    placeFreightIn = levels.MID;
                    break;
                case Right:
                    //IF BARCODE IS ON RIGHT SIDE
                    placeFreightIn = levels.MIN;
                    break;
                default:
                    placeFreightIn = levels.MAX;
                    break;
            }

            telemetry.addData("Barcode Location:", pipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if(withVision){
            webcam.stopStreaming();
        }

        drive.followTrajectorySequence(placement);
        /*switch (placeFreightIn)
        {
            case MIN:
                goToMin();
                break;
            case MID:
                goToMid();
                break;
            case MAX:
                    goToMax();
                    break;
        }*/

        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        Thread.sleep(1500);
        fixIntake();
        drive.followTrajectorySequence(cycle);
        //goToMax();
        drive.followTrajectorySequence(entrance2);
        intake.intakeForward();
        drive.followTrajectorySequence(collect2);
        Thread.sleep(1500);
        fixIntake();
        //threadAuto.interrupt();
        Thread.currentThread().interrupt();
    }

    /*void goToMin() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MIN);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO);
        }
    }

    void goToMid() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MID);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO);
        }
    }

    void goToMax() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MAX);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO);
        }
    }*/

    void fixIntake() throws InterruptedException {
        intake.intakeBackward();
        Thread.sleep((long)(reverseIntakeFor * 1000));
        intake.stop();
    }

    void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        HSVPipeline pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam,0);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}
