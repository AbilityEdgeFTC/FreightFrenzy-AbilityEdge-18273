package org.firstinspires.ftc.teamcode.opmodes.autonmous.t265;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.T265Localizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Right Red T265", group = "red")
@Disabled
public class AutoRightRed265 extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -60;
    double startPoseRightH = 90;
    public static double poseEntranceX = 17.5;
    public static double poseEntranceY = -62;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50;
    public static double poseCollectY = -62;
    public static double poseCollectH = 180;
    public static double poseHubX = 16.5;
    public static double poseHubY = -61;
    public static double poseHubH = 180;
    public static double poseHelpX = 9;
    public static double poseHelpY = -50;
    public static double poseHelpH = 180;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;
    T265Localizer t265Localizer;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    public static int elevatorLevel = 3;

    OpenCvWebcam webcam;
    HSVPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        t265Localizer = new T265Localizer(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));

        drive.setLocalizer(t265Localizer);

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                switch (placeFreightIn)
                {
                    case MIN:
                        elevatorLevel = 1;
                        break;
                    case MID:
                        elevatorLevel = 2;
                        break;
                    case MAX:
                        elevatorLevel = 3;
                        break;
                }

                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                canIntake = false;

                dip.holdFreight();

                switch (elevatorLevel)
                {
                    case 1:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                        hand.level1();
                        break;
                    case 2:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                        hand.level2();
                        break;
                    case 3:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                        hand.level3();
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorClose =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevator.updateAuto();
                spinner.updateAuto();
                canIntake = true;
                hand.intake();
                elevator.setPower(powerSlowElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeBackward();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        initPipeline();

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(7)
                .lineToLinearHeading(poseHelp)
                .lineToLinearHeading(poseHub)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .build();

        dip.getFreight();

        while (!opModeIsActive())
        {
            telemetry.addData("BARCODE LOCATION: ", pipeline.getLocation());
            switch (pipeline.getLocation())
            {
                case Left:
                    placeFreightIn = levels.MIN; // RED, blue = 3
                    break;
                case Center:
                    placeFreightIn = levels.MID; // RED, blue = 2
                    break;
                case Right:
                    placeFreightIn = levels.MAX; // RED, blue = 1
                    break;
            }
        }

        waitForStart();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);
    }

    public void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new HSVPipeline();
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
