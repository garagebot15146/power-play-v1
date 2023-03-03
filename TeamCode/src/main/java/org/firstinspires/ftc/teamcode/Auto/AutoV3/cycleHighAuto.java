package org.firstinspires.ftc.teamcode.Auto.AutoV3;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.commands.cycleHighCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

@Autonomous(name = "cycleHighAuto", group = "auto")
//@Disabled
public class cycleHighAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtendSubsystem extendSubsystem = new ExtendSubsystem(hardwareMap);

        CommandScheduler.getInstance().reset();

        waitForStart();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 5),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 4),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 3),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 2),
                        new cycleHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, 1),
                        new liftCommand(liftSubsystem, "HIGH"),
                        new WaitCommand(500),
                        new liftCommand(liftSubsystem, "BOTTOM")
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



