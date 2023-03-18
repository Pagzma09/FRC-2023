package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.LEDConstants.*;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LightsV2;

public class LightsV2_Commands {    
    public static Command ShowBatteryState(LightsV2 lightsvtwoer) {
        return lightsvtwoer.run(() -> {
            lightsvtwoer.sendCommand(LED_Commands.VOLTAGE, (byte) (
                (LEDConstants.numLEDS) / (LEDConstants.highBattery - LEDConstants.lowBattery) *
                (RobotController.getBatteryVoltage() - LEDConstants.lowBattery)
            ));
        }).ignoringDisable(true);
    }   

    // -expl Shows our alliance colour!
    public static Command showAllianceColour(LightsV2 lightsvtwoer){
        return lightsvtwoer.runOnce(() -> {
            lightsvtwoer.sendCommand(
                (DriverStation.getAlliance()==Alliance.Blue) ? LED_Commands.BLUE_ALLIANCE:LED_Commands.RED_ALLIANCE, (byte) 0
            );
        }).ignoringDisable(true).andThen(lightsvtwoer.run(() -> {})).ignoringDisable(true);
    }

    //-expl Give the human player a visual end-game warning secondsLeftBeforeEndGameWarning seconds before the game ends.
    public static Command showEndgameWarning(LightsV2 lightsvtwoer){
        Command previousCommand = lightsvtwoer.getCurrentCommand();

        return lightsvtwoer.runOnce(() -> {
            lightsvtwoer.sendCommand(LED_Commands.TIME_LEFT, (byte) DriverStation.getMatchTime());
        }).ignoringDisable(true)
        .andThen(Commands.waitSeconds(3))
        .andThen(Commands.runOnce(() -> {
            previousCommand.initialize();
            previousCommand.schedule();
        }).ignoringDisable(true));
    }

}
