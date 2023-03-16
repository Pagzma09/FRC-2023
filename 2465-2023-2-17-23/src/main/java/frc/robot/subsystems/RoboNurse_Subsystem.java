package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LEDConstants.*;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RoboNurse_Constants;

public class RoboNurse_Subsystem extends SubsystemBase{
    public RoboNurse_Subsystem(){}

    public CommandBase StartRoboNurse_Command(LED_Subsystem leds){

        return leds.run(() ->
        {
            leds.sendCommand(LED_Commands.SET_VOLTAGE, (byte) (
                ((LEDConstants.numLEDS) * (RobotController.getBatteryVoltage()/RoboNurse_Constants.highBattery)) +
                (LEDConstants.numLEDS) -
                ((LEDConstants.numLEDS * RoboNurse_Constants.highBattery)/(RoboNurse_Constants.highBattery 
                - RoboNurse_Constants.lowBattery))));
        });
    }   
}
