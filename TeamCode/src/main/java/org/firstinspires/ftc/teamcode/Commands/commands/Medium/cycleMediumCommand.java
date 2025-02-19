package org.firstinspires.ftc.teamcode.Commands.commands.Medium;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class cycleMediumCommand extends SequentialCommandGroup {
    public cycleMediumCommand(IntakeSubsystem intakeSubsystem , LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new depositMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem, cones),
                new WaitCommand(100),
                new transferMediumCommand(intakeSubsystem, liftSubsystem, extendSubsystem)
        );
    }
}
