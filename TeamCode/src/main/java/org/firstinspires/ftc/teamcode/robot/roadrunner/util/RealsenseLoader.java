package org.firstinspires.ftc.teamcode.robot.roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import javax.annotation.Nullable;

public class RealsenseLoader {

    public static final Transform2d cameraRobotOffset = PoseUtil.toTransform2d(
            PoseUtil.metersToInches(
                    new Pose2d(-0.065, 0.14, Math.toRadians(90))));
    public static final double encoderMeasurementCovariance = 0.5;

    @Nullable
    public static T265Camera slamera = null;

    public static void init(HardwareMap hardwareMap) {
        if (slamera == null) {
            slamera = new T265Camera(cameraRobotOffset, encoderMeasurementCovariance, hardwareMap.appContext);
            slamera.start();
        }
    }

    /**
     * Loads up the realsense camera
     */
    public RealsenseLoader(HardwareMap hardwareMap) {
        init(hardwareMap);
    }
}