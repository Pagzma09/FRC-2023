package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDInterface extends SubsystemBase{
    ArrayList<DigitalOutput> outputs;

    public AddressableLEDInterface(){}

    public void addChannel(int channel){
        outputs.add(new DigitalOutput(channel));
    }

    public void setValue(boolean value, int position){
        if(position < outputs.size()){
            outputs.get(position).set(value);
        }
    }

    public void setValueBin(int num){
        num %= Math.abs(Math.pow(2, outputs.size()) - 1);

        if(num < Math.pow(2, outputs.size()) - 1){
            for(int n = 0; n < outputs.size(); n++){
                this.setValue((((num >> n) & 1) == 1? true: false), n);
            }
        }
    }

    public int getValueBin(){
        int returnMe = 0;
        for(DigitalOutput output : outputs){
            returnMe += (output.get()? 1 : 0) * Math.pow(2, outputs.indexOf(output));
        }
        return returnMe;
    }

    public void periodic(){}
}
