package org.firstinspires.ftc.teamcode.Commands.commandBase.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Commands.commandBase.subsystem.LiftSubsystem;

public class liftCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;
    private int target;

    public liftCommand(LiftSubsystem subsystem, int target) {
        this.target = target;
        liftSubsystem = subsystem;
        addRequirements(liftSubsystem);
    }


    @Override
    public void initialize() {
        liftSubsystem.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        return liftSubsystem.isReached();
    }

}
