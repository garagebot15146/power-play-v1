package org.firstinspires.ftc.teamcode.Commands.commands.Medium;

import org.firstinspires.ftc.teamcode.Commands.commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

public class depositMediumCommand extends ParallelCommandGroup {
    public depositMediumCommand(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, int cones) {
        super(
                new intakeCommand(intakeSubsystem, cones),
                new liftCommand(liftSubsystem, "MEDIUM", 1100),
                new extendMediumCommand(extendSubsystem, cones, 3000)
        );
    }
}
