package org.firstinspires.ftc.teamcode.Commands.commands;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class cycleHighCommand extends SequentialCommandGroup {
    public cycleHighCommand(IntakeSubsystem intakeSubsystem , LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new depositCommand(intakeSubsystem, liftSubsystem, extendSubsystem, cones),
                new transferHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem)
        );
    }
}
