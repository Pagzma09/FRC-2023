package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LED_Commands;
import frc.robot.Constants.LEDConstants.LED_Modes;

public class LED_2023_Subsystem extends SubsystemBase {
    
    public LED_2023_Subsystem(){}

    // -expl Shows our alliance colour!
    public Command showAllianceColour_Command(LED_Subsystem leds){
        return leds.runOnce(() -> {
            leds.sendCommand(LED_Commands.CHANGE_MODE, (byte) (DriverStation.getAlliance() == Alliance.Blue? LED_Modes.BLUE_ALLIANCE.ordinal(): LED_Modes.RED_ALLIANCE.ordinal()));
        });
    }
    

    //-expl Shows our tilt on the seesaw (charging thingy).
    public Command showTilt_Command(LED_Subsystem leds, Drive drive){
        return leds.run(() -> {
            leds.sendCommand(LED_Commands.SET_TILT, (byte) drive.getPitch());
        });
    }

    //-expl Tells the *driver* that they need to hurry to tilt.
    public Command showEndgameWarning_Command(LED_Subsystem leds){
        return leds.runOnce(() -> {
            leds.sendCommand(LED_Commands.SET_TIME_LEFT, (byte) DriverStation.getMatchTime());
        });
    }
}
