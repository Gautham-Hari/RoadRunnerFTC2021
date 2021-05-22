package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        if(!isStopRequested())
        {
            drive.wobbleGoalArm.setTargetPosition(-550);
            drive.wobbleGoalArm.setPower(0.4);
        }

        drive.wobbleGripL.setPosition(1);
        drive.wobbleGripL.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;
        Pose2d startPose = new Pose2d(-60, 50, Math.toRadians(0));

        if(!isStopRequested())
        {
            drive.wobbleGoalArm.setTargetPosition(-1550);
            drive.wobbleGoalArm.setPower(0.4);
        }

        drive.setPoseEstimate(startPose);
        Trajectory toZoneA = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(2, 60), 0)
                .build();

        Trajectory toZoneC = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(50, 60), 0)
                .build();


        Trajectory secondWobble = drive.trajectoryBuilder(toZoneC.end(),true)
                .lineToLinearHeading(new Pose2d(-45, 50,Math.toRadians(-90)))
                .build();
        Trajectory forward = drive.trajectoryBuilder(secondWobble.end(),false)
                .forward(7)
                .build();
        Trajectory back = drive.trajectoryBuilder(forward.end())
                .back(14)
                .build();
//        Trajectory dropSecondA = drive.trajectoryBuilder(back.end(),false)
//                .splineToLinearHeading(new Pose2d(-10, 60,Math.toRadians(0)), Math.toRadians(-30))
//                .build();
        Trajectory dropSecondC = drive.trajectoryBuilder(back.end().plus(new Pose2d(0, 0, Math.toRadians(90))),false)
                .lineToConstantHeading(new Vector2d(45,60))
                .build();

//        Trajectory pickUpSecond = drive.trajectoryBuilder(back.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
//                .forward(10)
//                .build();
//        Trajectory backUp = drive.trajectoryBuilder(pickUpSecond.end(),true)
//                .forward(10)
//                .build();
//        Trajectory toZoneASecond = drive.trajectoryBuilder(backUp.end())
//                .splineTo(new Vector2d(2, 60), 0)
//                .build();

        drive.followTrajectory(toZoneC);
        sleep(250);

        drive.wobbleGripL.setPosition(0);
        drive.wobbleGripL.setPosition(1);

        drive.followTrajectory(secondWobble);
        drive.followTrajectory(forward);

        sleep(250);
        drive.wobbleGripL.setPosition(1);
        drive.wobbleGripL.setPosition(0);
        sleep(250);

        if(!isStopRequested())
        {
            drive.wobbleGoalArm.setTargetPosition(-550);
            drive.wobbleGoalArm.setPower(0.4);
        }

        drive.followTrajectory(back);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(dropSecondC);

        if(!isStopRequested())
        {
            drive.wobbleGoalArm.setTargetPosition(-1550);
            drive.wobbleGoalArm.setPower(0.4);
        }
        drive.wobbleGripL.setPosition(0);
        drive.wobbleGripL.setPosition(1);


//        drive.followTrajectory(pickUpSecond);
//        sleep(1000);
//        drive.followTrajectory(backUp);
//        sleep(1000);
//        drive.turn(Math.toRadians(90));
//        drive.followTrajectory(toZoneASecond);



stop();
    }
}
