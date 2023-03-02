package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.commandBase.commands.clawPosCommand;
import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commandBase.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.LiftSubsystem;

@Autonomous(name = "commandAuto", group = "auto")
//@Disabled
public class commandAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);

        CommandScheduler.getInstance().reset();

        waitForStart();
        CommandScheduler.getInstance().schedule(
                new clawPosCommand(clawSubsystem),
                new SequentialCommandGroup(
                        new liftCommand(liftSubsystem, 500),
                        new WaitCommand(500),
                        new liftCommand(liftSubsystem, 5),
                        new InstantCommand(this::requestOpModeStop)
                )
        );

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            liftSubsystem.loop();
            telemetry.addData("Lift", liftSubsystem.position());
            telemetry.update();
        }
    }

}


