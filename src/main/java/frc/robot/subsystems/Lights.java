package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.LightsDefault;

import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends SubsystemBase {
  private Spark aLeds;
  private Spark sLeds;
  private String fieldSide = "blue";
  private double aMode = LEDMode.RED.value;
  private double sMode = LEDMode.RED.value;
  private String lightMode = "side";
  private double startTime = Timer.getFPGATimestamp();

  public Lights() {
    aLeds = new Spark(1);
    sLeds = new Spark(0);
    // aLeds.setBounds(1.0, 0.8, 0.0, -0.8, -1.0);
  }

  public void setFieldSide(String side){
    this.fieldSide = side;
    setCorrectMode();
  }

  public void setLightsToIntake() {
    setAMode(LEDMode.GREENSTROBE);
    setSMode(LEDMode.GREENBPM);
  }

  public void setLightMode(String mode){
    startTime = Timer.getFPGATimestamp();
    lightMode = mode;
    setCorrectMode();
  }

  public void setCorrectMode(){
    if (Timer.getFPGATimestamp() - startTime > 10){
      this.lightMode = "side";
    }
    if (this.lightMode == "side"){
      if (this.fieldSide == "red"){
        setAMode(LEDMode.RED);
        setSMode(LEDMode.RED);
      } else if (this.fieldSide == "blue"){
        setAMode(LEDMode.BLUE);
        setSMode(LEDMode.BLUE);
      }
    } else if (this.lightMode == "cube"){
      setAMode(LEDMode.VIOLET);
      setSMode(LEDMode.VIOLET);
    } else if (this.lightMode == "cone"){
      setAMode(LEDMode.YELLOW);
      setSMode(LEDMode.YELLOW);
    }
  }

  // setMode method uses constants from the LEDMode enum
  public void setAMode(LEDMode mode){
    aMode = mode.value;
  }

  public void setSMode(LEDMode mode){
    sMode = mode.value;
  }
  
  public enum LEDMode{
    BLUE(0.87), RED(0.61), GREEN(0.75), YELLOW(0.67), RAINBOW(-0.97), OFF(0.99), ORANGE(0.65), REDFLASH(-0.11), VIOLET(0.91), REDONBLUE(0.53), BLUEONRED(0.41), WHITE(0.93), GREENSTROBE(0.35), GREENBPM(0.43), GOLDSTROBE(-0.07);

    public final double value;
    private LEDMode(double value){
        this.value = value;
    }
  }

  public void periodic() {
    // System.out.println("Setting mode: " + currentLedMode);
    aLeds.set(aMode);
    sLeds.set(sMode);
  }
  
  public void init() {
    setDefaultCommand(new LightsDefault(this));
  }

  public void autoInit() {

  }

  public void teleopInit() {
    setDefaultCommand(new LightsDefault(this));
  }
}
