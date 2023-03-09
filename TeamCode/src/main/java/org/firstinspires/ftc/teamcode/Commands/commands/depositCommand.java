package org.firstinspires.ftc.teamcode.Commands.commands;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class depositCommand extends ParallelCommandGroup {
    public depositCommand(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new intakeCommand(intakeSubsystem, cones),
                new liftCommand(liftSubsystem, "HIGH", 1000),
                new extendCommand(extendSubsystem, cones, 1100)
        );
    }
}
