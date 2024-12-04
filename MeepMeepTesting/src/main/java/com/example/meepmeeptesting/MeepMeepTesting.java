package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.ColorScheme;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static int Side_Coeff = -1;
    public static  int Sample_Degs = 90*Side_Coeff;
    static public Vector2d Basket_Position = new Vector2d(-60*Side_Coeff,-60*Side_Coeff);
    static public Vector2d FieldEdge = new Vector2d(-75*Side_Coeff,-75*Side_Coeff);
    static double Basket_Degs = Math.atan2(FieldEdge.getY() - Basket_Position.getY(), FieldEdge.getX() - Basket_Position.getX()) * 180 / Math.PI;

    public static void main(String[] args) {
        MeepMeep SampleMeepMeep = new MeepMeep(600);
        MeepMeep SpeceminMeepMeep = new MeepMeep(600);

        RoadRunnerBotEntity SampleBot = new DefaultBotBuilder(SampleMeepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(312, 200, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60*Side_Coeff, -60*Side_Coeff, Math.toRadians(90*Side_Coeff)))
                        .lineToLinearHeading(new Pose2d(0,-30*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(2)
                        .back(10)
                        //Sample 1
                        .lineToLinearHeading(new Pose2d(-48*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)
                        //Sample 2
                        .lineToLinearHeading(new Pose2d(-58*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)
                        //Sample 3
                        .lineToLinearHeading(new Pose2d(-68*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(-60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)
                        //Park in Observation
                        .lineToLinearHeading(new Pose2d(60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Sample_Degs)))

                        .build()
                );

        RoadRunnerBotEntity SpeceminBot = new DefaultBotBuilder(SpeceminMeepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(312, 200, Math.toRadians(180), Math.toRadians(180), 15)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60*Side_Coeff, -60*Side_Coeff, Math.toRadians(90*Side_Coeff)))
                        .lineToLinearHeading(new Pose2d(0,-30*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(2)
                        .back(10)
                        //Sample 1
                        .lineToLinearHeading(new Pose2d(48*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)

                        //Deposit Specemin 1
                        .lineToLinearHeading(new Pose2d(0,-30*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(2)
                        .back(10)

                        //Sample 2
                        .lineToLinearHeading(new Pose2d(58*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)

                        //Deposit Specemin 2
                        .lineToLinearHeading(new Pose2d(0,-30*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(2)
                        .back(10)

                        //Sample 3
                        .lineToLinearHeading(new Pose2d(68*Side_Coeff,-35*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(60*Side_Coeff,-60*Side_Coeff,Math.toRadians(Basket_Degs)))
                        .waitSeconds(1)

                        //Deposit Specemin 3
                        .lineToLinearHeading(new Pose2d(0,-30*Side_Coeff,Math.toRadians(Sample_Degs)))
                        .waitSeconds(2)
                        .back(10)

                        //Park in Observation
                        .lineToLinearHeading(new Pose2d(60*Side_Coeff,-60*Side_Coeff,Math.toRadians(-Sample_Degs)))

                        .build()
                );



        SampleMeepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(SampleBot)
                .setShowFPS(true)
                .setTheme(new ColorSchemeBlueDark())
                .start();

        SpeceminMeepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(SpeceminBot)
                .setShowFPS(true)
                .setTheme(new ColorSchemeBlueDark())
                .start();

    }
}