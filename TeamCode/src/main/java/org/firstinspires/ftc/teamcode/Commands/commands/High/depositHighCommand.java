package org.firstinspires.ftc.teamcode.Commands.commands.High;

import org.firstinspires.ftc.teamcode.Commands.commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

public class depositHighCommand extends ParallelCommandGroup {
    public depositHighCommand(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new intakeCommand(intakeSubsystem, cones),
                new liftCommand(liftSubsystem, "HIGH", 2000),
                new extendHighCommand(extendSubsystem, cones, 3000)
        );
    }
}
