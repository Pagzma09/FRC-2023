package frc.robot.subsystems;

import java.security.InvalidParameterException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class AddressableLEDInterface extends SubsystemBase{
    ArrayList<DigitalOutput> outputs;
    public static enum LEDCommands {CycleBack, CycleFwd};

    public AddressableLEDInterface(){
        for(int n = 0; n < LEDConstants.numChannels; n++) outputs.add(new DigitalOutput(n + LEDConstants.startChannel));
    }

    public void addChannel(int channel){
        //-expl Adds a new output at the specified channel
        outputs.add(new DigitalOutput(channel));
    }

    public void setValue(boolean value, int position){
        //-expl Makes sure that the output is inside the desired range, then sets it.
        if(position < outputs.size()){
            outputs.get(position).set(value);
        } else throw new InvalidParameterException("Position out of range for current channel selection");
    }

    public void setValueBin(int num){
        //-expl Sets the outputs to be a binary number.
        for(int n = 0; n < outputs.size(); n++){
            this.setValue((((num >> n) & 1) == 1? true: false), n);
        }
    }

    public int getValueBin(){
        //-expl Gets the binary value from the outputs.
        int returnMe = 0;
        for(DigitalOutput output : outputs){
            //-expl For every output, add 2^(number of output) if output is one. For example, 1001 -> 1 * 2^0 + 0 * 2^1 + 0 * 2^2 + 1 * 2^4 -> 1 + 4 -> 5
            returnMe += (output.get()? 1 : 0) * 1 << outputs.indexOf(output);
        }
        return returnMe;
    }

    public boolean getValue(int position){
        //-expl Gets the value from a certain output. If out of range, throw exception.
        if(position < outputs.size()){
            return outputs.get(position).get();
        } else throw new InvalidParameterException("Position out of range for current channel selection");
    }

    public void periodic(){}
}
