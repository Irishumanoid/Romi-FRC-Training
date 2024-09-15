package frc.robot.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class UltrasonicSensor {
  private final DigitalOutput trigPin;
  private final DigitalInput echoPin;
  private final Timer timer;
  private static final double ECHO_TIMEOUT = 0.03;

  public UltrasonicSensor(int trigPinChannel, int echoPinChannel) {
    this.trigPin = new DigitalOutput(trigPinChannel);
    this.echoPin = new DigitalInput(echoPinChannel);
    timer = new Timer();
  }

  public boolean isObjectTooClose() {
    trigPin.set(false);
    Timer.delay(0.00001);
    trigPin.set(true);
    Timer.delay(0.00001);
    trigPin.set(false);

    timer.reset();
    timer.start();
    while (!echoPin.get()) {
      if (timer.get() > ECHO_TIMEOUT) {
        timer.stop();
        return false;
      }
    }

    timer.reset();
    timer.start();
    while (echoPin.get()) {
      if (timer.get() > ECHO_TIMEOUT) {
        timer.stop();
        return false;
      }
    }
    timer.stop();

    double timeElapsed = timer.get();
    double distanceCM = (timeElapsed * 34300) / 2;
    return distanceCM < 5;
  }
}
