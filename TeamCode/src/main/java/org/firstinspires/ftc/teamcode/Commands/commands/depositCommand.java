package org.firstinspires.ftc.teamcode.Commands.commands;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

public class depositCommand extends ParallelCommandGroup {
    public depositCommand(IntakeSubsystem intakeSubsystem , LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new liftCommand(liftSubsystem, "HIGH"),
                new intakeCommand(intakeSubsystem, cones),
                new extendCommand(extendSubsystem, 960)
        );
    }
}
