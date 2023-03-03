package org.firstinspires.ftc.teamcode.Commands.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.subsystem.LiftSubsystem;

public class liftCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private String pole;

    public liftCommand(LiftSubsystem subsystem, String pole) {
        this.pole = pole;
        liftSubsystem = subsystem;
        addRequirements(liftSubsystem);
    }


    @Override
    public void initialize() {
        liftSubsystem.setTarget(pole);
    }

    @Override
    public boolean isFinished() {
        return liftSubsystem.isReached();
    }

}
