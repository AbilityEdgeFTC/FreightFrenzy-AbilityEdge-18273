package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMLeftRed {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseLeftX = -35;
        double startPoseLeftY = -60;
        double startPoseLeftH = 90;
        double poseCarouselX = -59.5;
        double poseCarouselY = -57.5;
        double poseCarouselH = 95;



        double carouselHelp = 15;


        double poseParkHelpX = -27.5;
        double poseParkHelpY = -6;
        double poseParkHelpH = 180;

        double poseParkaX = 7.5;
        double poseParkaY = -9;
        double poseParkaH = 180;

        double poseParkbX = 7.5;
        double poseParkbY = -45;
        double poseParkbH = 180;

        double poseParkcX = 50;
        double poseParkcY = -45;
        double poseParkcH = 180;

        double runCarouselFor = 4;
        //Pose2d turnPoseRight = new Pose2d(turnPoseRightX,turnPoseRightY,Math.toRadians(turnPoseRightH));
        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(270), 12.805)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .waitSeconds( 1.5)
                                .forward(carouselHelp)
                                .lineToLinearHeading(poseCarousel)
                                .waitSeconds(runCarouselFor)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .lineToSplineHeading(poseParkingHelp)
                                .splineToLinearHeading(poseParkinga,poseParkaH)
                                .lineToLinearHeading(poseParkingb)
                                .waitSeconds(.6)
                                .lineToLinearHeading(poseParkingc)
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