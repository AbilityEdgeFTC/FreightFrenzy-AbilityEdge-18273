package org.firstinspires.ftc.teamcode.robot.roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public class PoseUtil {
    private static final double metersToInchesMultiplier = 100.0/2.54;

    /**
     * Converts a FTCLib pose to a Roadrunner pose
     * @param inputPose FTCLib pose to convert
     * @return Roadrunner pose with the same value as the input
     */
    public static Pose2d toRoadrunnerPose(com.arcrobotics.ftclib.geometry.Pose2d inputPose){
        return new Pose2d(inputPose.getX(), inputPose.getY(), inputPose.getHeading());
    }

    /**
     * Converts a Roadrunner pose to a FTCLib pose
     * @param inputPose Roadrunner pose to convert
     * @return FTCLib pose with the same value as the input
     */
    public static com.arcrobotics.ftclib.geometry.Pose2d toFtclibPose(Pose2d inputPose){
        return new com.arcrobotics.ftclib.geometry.Pose2d(
                inputPose.getX(),
                inputPose.getY(),
                new Rotation2d(inputPose.getHeading()));
    }

    /**
     * Converts measurements from meters to inches
     * @param inputPose pose to convert
     * @return converted pose
     */
    public static Pose2d metersToInches(Pose2d inputPose) {
        return new Pose2d(
                inputPose.vec().times(metersToInchesMultiplier),
                inputPose.getHeading());
    }

    /**
     * Converts measurements from inches to meters
     * @param inputPose pose to convert
     * @return converted pose
     */
    public static Pose2d inchesToMeters(Pose2d inputPose) {
        return new Pose2d(
                inputPose.vec().div(metersToInchesMultiplier),
                inputPose.getHeading());
    }

    /**
     * Converts measurements from inches to meters
     * @param inputPose pose to convert
     * @return converted pose
     */
    public static Translation2d inchesToMeters(Translation2d inputPose) {
        return new Translation2d(inputPose.div(metersToInchesMultiplier).getX(), inputPose.div(metersToInchesMultiplier).getY());
    }

    /**
     * Converts a ChassisSpeeds object to a Roadrunner Pose2d
     * @param chassisSpeeds ChassisSpeeds object to convert
     * @return Roadrunner pose with the same value
     */
    public static Pose2d chassisSpeedsToRoadrunnerPose(ChassisSpeeds chassisSpeeds) {
        return new Pose2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Converts a Roadrunner Pose2d to a FTCLib Transform2d
     * @param inputPose Pose to convert
     * @return Transform2d with the same value
     */
    public static Transform2d toTransform2d(Pose2d inputPose) {
        return new Transform2d(
                new Translation2d(inputPose.getX(), inputPose.getY()),
                new Rotation2d(inputPose.getHeading()));
    }

    /**
     * Multiply each individual component
     * @param inputPose Pose to multiply
     * @param xFactor Factor of the x component
     * @param yFactor Factor of the y component
     * @param headingFactor Factor of the heading component
     * @return Multiplied pose
     */
    public static Pose2d multiply(Pose2d inputPose, double xFactor, double yFactor, double headingFactor) {
        return new Pose2d(inputPose.getX() * xFactor, inputPose.getY() * yFactor, inputPose.getHeading() * headingFactor);
    }

    /**
     * Check whether each of the position values is greater than another value
     * @param inputPose Pose to check
     * @param x X "other" value
     * @param y Y "other" value
     * @return Whether each component is greater than its "other" value
     */
    public static boolean greaterThan(Pose2d inputPose, double x, double y) {
        return inputPose.getX() > x && inputPose.getY() > y;
    }

    /**
     * Get the absolute value of each individual component
     * @param inputPose Pose to get the absolute value of
     * @return Absolute valued pose
     */
    public static Pose2d abs(Pose2d inputPose) {
        return new Pose2d(
                Math.abs(inputPose.getX()),
                Math.abs(inputPose.getY()),
                inputPose.getHeading());
    }
}