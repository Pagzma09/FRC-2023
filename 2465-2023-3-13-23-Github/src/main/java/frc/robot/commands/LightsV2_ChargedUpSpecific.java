package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LEDConstants.LED_Commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LightsV2;
import static frc.robot.subsystems.LightsV2.*;

public class LightsV2_ChargedUpSpecific {
        //-expl Shows our tilt on the seesaw (charging thingy).
    public static Command showTilt(LightsV2 lightsvtwoer, Drive driver) {
        return lightsvtwoer.run(() -> {
            lightsvtwoer.sendCommand(LED_Commands.TILT, (byte) driver.getPitch());
        }).ignoringDisable(true);
    }
  
    public static Command requestCube(LightsV2 lightsvtwoer) {
        return Commands.runOnce(() -> {
            lightsvtwoer.updateCurrentCommand();
            lightsvtwoer.runOnce(() -> {
                System.out.println("Req'd cube");
                lightsvtwoer.sendCommand(LED_Commands.REQUEST_CUBE, (byte) 0);
            })
            .andThen(Commands.waitSeconds(3))
            .andThen(Commands.runOnce(() -> {
                LightsV2.continuePrevious();
            }, lightsvtwoer).ignoringDisable(true)).withName("x").schedule();
        }).withName("x");
    }

    public static Command requestCone(LightsV2 lightsvtwoer) {
        return Commands.runOnce(() -> {
            lightsvtwoer.updateCurrentCommand();
            Command previousCommand = lightsvtwoer.getCurrentCommand();
            lightsvtwoer.runOnce(() -> {
                System.out.println("Req'd cone");
                lightsvtwoer.sendCommand(LED_Commands.REQUEST_CONE, (byte) 0);
            })
            .andThen(Commands.waitSeconds(3))
            .andThen(Commands.runOnce(() -> {
                LightsV2.continuePrevious();
            }, lightsvtwoer).ignoringDisable(true)).withName("x").schedule();
        }).withName("x");
    }
}
