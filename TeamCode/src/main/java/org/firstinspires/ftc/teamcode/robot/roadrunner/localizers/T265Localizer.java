package org.firstinspires.ftc.teamcode.robot.roadrunner.localizers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class T265Localizer implements Localizer
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    T265Camera.CameraUpdate up;
    public static double X = 0;
    public static double Y = 0;

    public T265Localizer(HardwareMap hardwareMap)
    {
        Translation2d translation2d = new Translation2d(X / 100, Y / 100);
        Rotation2d rotation2d = new Rotation2d(Math.toRadians(0));
        if(slamra == null)
        {
            slamra = new T265Camera(new Transform2d(translation2d, rotation2d), 0.1, hardwareMap.appContext);
        }
        start();
    }

    public void start() {
        slamra.start();
    }

    public void stop() {
        slamra.stop();
    }

    @Override
    public @NotNull Pose2d getPoseEstimate() {
        up = slamra.getLastReceivedCameraUpdate();
        return new Pose2d(-up.pose.getY() / 0.0254, up.pose.getX() / 0.0254, up.pose.getHeading());
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        up = slamra.getLastReceivedCameraUpdate();
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getY() * 0.0254, -pose2d.getX() * 0.0254, new Rotation2d(pose2d.getHeading())));
    }

    @Override
    public @Nullable Pose2d getPoseVelocity() {
        up = slamra.getLastReceivedCameraUpdate();
        return new Pose2d(-up.velocity.vyMetersPerSecond / 0.0254, up.velocity.vxMetersPerSecond / 0.0254, up.velocity.omegaRadiansPerSecond);
    }

    @Override
    public void update() {
        up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;
    }

    public void sendOdometry(Pose2d velocity) {
        slamra.sendOdometry(velocity.getX(), velocity.getY());
    }
}