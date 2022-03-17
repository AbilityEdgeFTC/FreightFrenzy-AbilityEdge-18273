package org.firstinspires.ftc.teamcode.robot.roadrunner.localizers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import java.util.List;

/**
 * Localizer using both the tracking wheels and the realsense camera
 * DOES NOT WORK - HASN'T BEEN TESTED!!!!!
 */
public class DoubleLocalizer implements Localizer {
    private final T265Localizer realsenseLocalizer;
    private SampleMecanumDrive mecanumLocalizer;

    private ConfidenceTracker confidenceTracker = ConfidenceTracker.HIGH;

    /**
     * Keeps track of what the latest confidence was. If the confidence just turned to HIGH,
     *  it will update the camera's pose before going into HIGH.
     */
    private enum ConfidenceTracker {
        LOW,
        HIGH
    }

    /**
     * Initializes both localizers
     * @param hardwareMap Hardware map passed in from an op mode
     */
    public DoubleLocalizer(HardwareMap hardwareMap) {
        super();
        realsenseLocalizer = new T265Localizer(hardwareMap);
        mecanumLocalizer = new SampleMecanumDrive(hardwareMap);
    }

    /**
     * Gets the most confident localizer's estimated position
     * @return The position
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        if (confidenceTracker == ConfidenceTracker.HIGH) {
            return realsenseLocalizer.getPoseEstimate();
        }
        return mecanumLocalizer.getPoseEstimate();
    }

    /**
     * Sets both localizers' estimated positions
     * @param pose2d The position
     */
    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        mecanumLocalizer.setPoseEstimate(pose2d);
        realsenseLocalizer.setPoseEstimate(pose2d);
    }

    /**
     * Gets the velocity of the current pose
     * @return The velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        // Using the camera's pose velocity because it takes the
        // tracking wheels into consideration when generating it
        return realsenseLocalizer.getPoseVelocity();
    }

    /**
     * Applies the tracking wheel velocity to the realsense camera and the updates
     *  the least confident localizer with the most confident one
     */
    @Override
    public void update() {
        List<Double> wheelVelocities = mecanumLocalizer.getWheelVelocities();
        realsenseLocalizer.sendOdometry(new Pose2d(wheelVelocities.get(0) * 0.0254, wheelVelocities.get(2) * 0.0254));

        // Get an update from the camera
        T265Camera.CameraUpdate update = realsenseLocalizer.up;

        // Update the pose of the least confident localizer
        switch (update.confidence) {
            case Failed:
            case Low:
            case Medium:
                confidenceTracker = ConfidenceTracker.LOW;
                break;
            default:
                if (confidenceTracker == ConfidenceTracker.LOW) {
                    realsenseLocalizer.setPoseEstimate(mecanumLocalizer.getPoseEstimate());
                    confidenceTracker = ConfidenceTracker.HIGH;
                }
        }
    }

    public void stop()
    {
        realsenseLocalizer.stop();
    }

}
