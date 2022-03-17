package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMRightRed {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 13;
        double startPoseRightY = -60;
        double startPoseRightH = 90;
        double poseEntranceX = 17.5;
        double poseEntranceY = -62;
        double poseEntranceH = 180;
        double poseCollectX = 50;
        double poseCollectY = -62;
        double poseCollectH = 180;
        double poseHubX = 16.5;
        double poseHubY = -61;
        double poseHubH = 180;
        double poseHelpX = 7;
        double poseHelpY = -50;
        double poseHelpH = 180;

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 12.805)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .lineToLinearHeading(poseHelp)
                                .lineToLinearHeading(poseEntrance)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .splineToLinearHeading(poseHub,poseHubH)
                                .waitSeconds(.8)
                                .splineToLinearHeading(poseEntrance,poseEntranceH)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .splineToLinearHeading(poseHub,poseHubH)
                                .waitSeconds(.8)
                                .splineToLinearHeading(poseEntrance,poseEntranceH)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .splineToLinearHeading(poseHub,poseHubH)
                                .waitSeconds(.8)
                                .splineToLinearHeading(poseEntrance,poseEntranceH)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .splineToLinearHeading(poseHub,poseHubH)
                                .waitSeconds(.8)
                                .splineToLinearHeading(poseEntrance,poseEntranceH)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .splineToLinearHeading(poseHub,poseHubH)
                                .waitSeconds(.8)
                                .splineToLinearHeading(poseEntrance,poseEntranceH)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .build()
                );

        myBot.setDimensions(13,17.9);
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}