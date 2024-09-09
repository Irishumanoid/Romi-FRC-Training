package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
  private final DigitalOutput redLed;
  private final DigitalOutput yellowLed;
  private final DigitalOutput greenLed;
  private int redFreq = 1;
  private int yellowFreq = 1;
  private int greenFreq = 1;
  private boolean blink = false;

  private final Timer timer;

  public LEDSubsystem() {
    redLed = new DigitalOutput(2);
    yellowLed = new DigitalOutput(3);
    greenLed = new DigitalOutput(4);
    timer = new Timer();
    timer.start();
  }

  public void setRedLedState(BooleanSupplier red) {
    redLed.set(red.getAsBoolean());
  }

  public void setYellowLedState(BooleanSupplier yellow) {
    yellowLed.set(yellow.getAsBoolean());
  }

  public void setGreenLedState(BooleanSupplier green) {
    greenLed.set(green.getAsBoolean());
  }

  public void setLedState(BooleanSupplier red, BooleanSupplier yellow, BooleanSupplier green) {
    setRedLedState(red);
    setYellowLedState(yellow);
    setGreenLedState(green);
  }

  @Override
  public void periodic() {
    if (blink) {
      double elapsedTime = timer.get();
      setRedLedState(() -> ((elapsedTime * redFreq) % 1) < 0.5);
      setYellowLedState(() -> ((elapsedTime * yellowFreq) % 1) < 0.5);
      setGreenLedState(() -> ((elapsedTime * greenFreq) % 1) < 0.5);
    }
  }

  public void setAutoBlinkState(BooleanSupplier state) {
    this.blink = state.getAsBoolean();
  }

  public void setRedBlinkFreq(int freq) {
    this.redFreq = freq;
  }

  public void setYellowBlinkFreq(int freq) {
    this.yellowFreq = freq;
  }

  public void setGreenBlinkFreq(int freq) {
    this.greenFreq = freq;
  }
}
