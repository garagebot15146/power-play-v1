package org.firstinspires.ftc.teamcode.Auto.AutoV3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.commands.Medium.cycleMediumCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;
import org.firstinspires.ftc.teamcode.Settings.trajectorysequence.TrajectorySequence;

@Autonomous(name = "leftMediumAuto", group = "auto")
@Config
@Disabled
public class leftMediumAuto extends LinearOpMode {
    HWMap drive;
    public static double toPoleBack = 34;
    public static double toPoleLineX = -30;
    public static double toPoleLineY = -25;
    public static double toPoleLineH = 168;

    public static double parkCenterLineX = -33;
    public static double parkCenterLineY = -16.5;
    public static double parkCenterLineH = 180;


    public static double parkLeftMove = 23;
    public static double parkLeftTurn = 90;

    public static double parkRightMove = 23;
    public static double parkRightTurn = 90;

    private ElapsedTime timer = new ElapsedTime();;


    @Override
    public void runOpMode() {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtendSubsystem extendSubsystem = new ExtendSubsystem(hardwareMap);
        drive = new HWMap(hardwareMap);

        Pose2d startPose = new Pose2d(-34, -72 + (15.5 / 2), Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence toPole = drive.trajectorySequenceBuilder(startPose)
                .back(toPoleBack)
                .lineToLinearHeading(new Pose2d(toPoleLineX, toPoleLineY, Math.toRadians(toPoleLineH)))
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .back(parkLeftMove)
                .turn(Math.toRadians(parkLeftTurn))
                .back(10)
                .build();

        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .turn(Math.toRadians(90))
                .back(10)
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(toPole.end())
                .lineToLinearHeading(new Pose2d(parkCenterLineX, parkCenterLineY, Math.toRadians(parkCenterLineH)))
                .forward(parkRightMove)
                .turn(Math.toRadians(parkRightTurn))
                .back(10)
                .build();

        CommandScheduler.getInstance().reset();

        waitForStart();
        timer.reset();
        drive.followTrajectorySequence(toPole);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new cycleMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 5),
                        new cycleMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 4),
                        new cycleMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 3),
                        new cycleMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 2),
                        new cycleMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 1),
                        new InstantCommand(() -> intakeSubsystem.down(0)),
                        new WaitCommand(100),
                        new liftCommand(liftSubsystem, "MEDIUM", 1200),
                        new liftCommand(liftSubsystem, "BOTTOM", 1000)
                )
        );

        while (opModeIsActive() && timer.seconds() < 25) {
            CommandScheduler.getInstance().run();
            liftSubsystem.loop();
            extendSubsystem.loop();
            telemetry.addData("Lift", liftSubsystem.position());
            telemetry.addData("Extend", extendSubsystem.position());
            telemetry.update();
        }
        drive.followTrajectorySequence(parkCenter);

    }

}



