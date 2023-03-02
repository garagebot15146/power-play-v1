package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.commandBase.commands.extendCommand;
import org.firstinspires.ftc.teamcode.Commands.commandBase.commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commandBase.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.LiftSubsystem;

@Autonomous(name = "commandAuto", group = "auto")
//@Disabled
public class commandAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem clawSubsystem = new IntakeSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtendSubsystem extendSubsystem = new ExtendSubsystem(hardwareMap);

        CommandScheduler.getInstance().reset();

        waitForStart();
        CommandScheduler.getInstance().schedule(
                new intakeCommand(clawSubsystem),
                new SequentialCommandGroup(
                        new liftCommand(liftSubsystem, 500),
                        new WaitCommand(500),
                        new liftCommand(liftSubsystem, 5)
                ),
                new SequentialCommandGroup(
                        new extendCommand(extendSubsystem, 500),
                        new WaitCommand(500),
                        new extendCommand(extendSubsystem, 5)
                )
        );

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            liftSubsystem.loop();
            extendSubsystem.loop();
            telemetry.addData("Lift", liftSubsystem.position());
            telemetry.addData("Extend", extendSubsystem.position());
            telemetry.update();
        }
    }

}


