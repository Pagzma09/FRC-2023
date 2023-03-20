package frc.robot.subsystems;

import static frc.robot.Constants.LEDConstants.LED_Port;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LED_Commands;

public class LightsV2 extends SubsystemBase {

    private static SPI LED_SPI = new SPI(LED_Port);


    public LightsV2 () {
        LED_SPI.setMode(SPI.Mode.kMode0);
        LED_SPI.setClockRate(500000);
        LED_SPI.setChipSelectActiveLow();
    }

    // -expl Every transaction consists of two bytes: a command, and a value. The value is based on the command, and the command is carefully chosen to convey a meaning.

    // set voltage = 12
    // set voltage = 11.9
    // set voltage = 11.8
    // ...
    // set mode = red_team
    // ...
    // set tilt = 11
    // set tilt = 0
    // ..

    byte[] byteBuffer = new byte[4];
    public void sendCommand(LED_Commands command, byte value) {
        byteBuffer[0] = (byte) LED_Commands.SYNC.ordinal();
        byteBuffer[1] = (byte) LED_Commands.SYNC.ordinal();
        byteBuffer[2] = (byte) (command.ordinal());
        byteBuffer[3] = value;

        LED_SPI.write(byteBuffer, 4);
        
        //for(byte Byte : byteBuffer){
        //    System.out.println(Byte);
        //}
    }

    
}