package org.firstinspires.ftc.teamcode.Commands.commands.High;

import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class cycleHighCommand extends SequentialCommandGroup {
    public cycleHighCommand(IntakeSubsystem intakeSubsystem , LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem, DistanceSensorSubsystem distanceSensorSubsystem, int cones) {
        super(
                new depositHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem,  cones),
                new transferHighCommand(intakeSubsystem, liftSubsystem, extendSubsystem, distanceSensorSubsystem)
        );
    }
}
