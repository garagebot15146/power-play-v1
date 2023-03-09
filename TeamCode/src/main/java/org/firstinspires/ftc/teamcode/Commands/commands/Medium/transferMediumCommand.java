package org.firstinspires.ftc.teamcode.Commands.commands.Medium;

import org.firstinspires.ftc.teamcode.Commands.commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Commands.commands.liftCommand;
import org.firstinspires.ftc.teamcode.Commands.subsystem.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

public class transferMediumCommand extends ParallelCommandGroup {
    public transferMediumCommand(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, ExtendSubsystem extendSubsystem) {
        super(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(300),
                        new extendMediumCommand(extendSubsystem, 6, 1300),
                        new WaitCommand(100),
                        new InstantCommand(() -> intakeSubsystem.lift()),
                        new WaitCommand(300),
                        new InstantCommand(() -> intakeSubsystem.update(IntakeSubsystem.RotatorState.DROP)),
                        new InstantCommand(() -> intakeSubsystem.update(IntakeSubsystem.WristPos.MID_CONE)),
                        new extendMediumCommand(extendSubsystem, 0, 1300)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> extendSubsystem.position() < 10),
                        new WaitCommand(100),
                        new intakeCommand(intakeSubsystem, 6),
                        new WaitCommand(500),
                        new InstantCommand(() -> intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(500)
                ),
                new liftCommand(liftSubsystem, "BOTTOM", 500)
        );
    }
}
