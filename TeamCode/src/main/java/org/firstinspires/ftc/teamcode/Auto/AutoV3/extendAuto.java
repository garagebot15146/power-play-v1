package org.firstinspires.ftc.teamcode.Auto.AutoV3;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.commands.High.extendHighCommand;
import org.firstinspires.ftc.teamcode.Commands.commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Settings.drive.HWMap;

@Autonomous(name = "extendAuto", group = "auto")
@Config
@Disabled
public class extendAuto extends LinearOpMode {
    HWMap drive;
    private ElapsedTime timer = new ElapsedTime();;

    @Override
    public void runOpMode() {
        ExtendSubsystem extendSubsystem = new ExtendSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        drive = new HWMap(hardwareMap);

        CommandScheduler.getInstance().reset();

        waitForStart();
        timer.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new intakeCommand(intakeSubsystem, 1),
                        new WaitCommand(500),
                        new extendHighCommand(extendSubsystem, 5, 10000),
                        new WaitCommand(500)
                )
        );

        while (opModeIsActive() && timer.seconds() < 25) {
            CommandScheduler.getInstance().run();
            extendSubsystem.loop();
            telemetry.addData("Extend", extendSubsystem.position());
            telemetry.addData("Distance", extendSubsystem.distance());
            telemetry.update();
        }
    }

}