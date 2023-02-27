package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.AddressableLEDInterface;
import frc.robot.subsystems.AddressableLEDInterface.LEDCommands;

public class AddressLEDs extends CommandBase{

    private final AddressableLEDInterface LEDer = Robot.aledi;
    private LEDCommands state;

    public AddressLEDs(LEDCommands state){
        addRequirements(LEDer);
        this.state = state;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        switch(state){
            case CycleBack:
                LEDer.setValueBin((LEDer.getValueBin() - 1 <= 0? 0: LEDer.getValueBin() - 1));
                break;
            case CycleFwd:
                LEDer.setValueBin((LEDer.getValueBin() + 1 > LEDConstants.numPatterns - 1? LEDConstants.numPatterns - 1: LEDer.getValueBin() + 1));
                break;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
