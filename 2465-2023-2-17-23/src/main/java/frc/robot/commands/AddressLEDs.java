package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AddressableLEDInterface;

public class AddressLEDs extends CommandBase{

    private AddressableLEDInterface LEDer = new AddressableLEDInterface();
    
    public AddressLEDs(int numChannels, int startChannel){
        addRequirements(LEDer);
        for(int n = startChannel; n - startChannel <= numChannels; n++){
            LEDer.addChannel(n);
        }
    }

    public void setPatternID(int patternID){
        LEDer.setValueBin(patternID);
    }

    public void cyclePatternFwd(){
        this.setPatternID(LEDer.getValueBin() + 1);
    }

    public void cyclePatternBkwd(){
        this.setPatternID(LEDer.getValueBin() - 1);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
