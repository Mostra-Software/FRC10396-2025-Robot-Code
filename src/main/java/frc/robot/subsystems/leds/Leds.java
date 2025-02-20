// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Optional;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean elevator_moving = false;
  public boolean intaking = false;
  public boolean hasCoral = false;
  public boolean autoDrive = false;
  public boolean climbing = false;
  public boolean endgameAlert = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean demoMode = false;
  public boolean leftCamDisconnected = false;
  public boolean rightCamDisconnected = false;
  public boolean leftReefSelected = false;
  public boolean rightReefSelected = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int ledFullLength = 100;
  private static final int ledLeftLength = 40;
  private static final int ledMiddleLenght = 20;
  private static final int ledRightLength = 40;
  private static final double strobeDuration = 0.1;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private Leds() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(ledFullLength);
    leds.setLength(ledFullLength);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    Color.kWhite,
                    Color.kBlack,
                    System.currentTimeMillis() / 1000.0,
                    0,
                    ledFullLength);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(Color.kGold);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;

      if (loadingNotifier != null) {
        loadingNotifier.stop();
      }
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    solid(Color.kBlack, 0, ledFullLength); // Default to off
    if (estopped) {
      solid(Color.kRed, 0, ledFullLength);
    } else if (DriverStation.isDisabled()) {
      if (leftCamDisconnected) {
        selektor(true, Color.kRed);
      } else if (rightCamDisconnected) {
        selektor(false, Color.kRed);
      } else if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(
            1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime),
            Color.kGreen,
            0,
            ledFullLength);
      } else if (lowBatteryAlert) {
        // Low battery
        solid(Color.kOrangeRed, 0, ledFullLength);
      } else {
        // Default pattern
        wave(
            allianceColor,
            secondaryDisabledColor,
            waveAllianceCycleLength,
            waveAllianceDuration,
            0,
            ledFullLength);
      }

      // Same battery alert
      if (DriverStation.isAutonomous()) {
        wave(Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration, 0, ledFullLength);
        if (autoFinished) {
          double fullTime = (double) ledFullLength / waveFastCycleLength * waveFastDuration;
          solid(
              (Timer.getFPGATimestamp() - autoFinishedTime) / fullTime,
              Color.kGreen,
              0,
              ledFullLength);
        }
      } else { // Enabled

        if (hasCoral) {
          strobe(Color.kGreen, strobeDuration, ledLeftLength, ledMiddleLenght);
        } else if (intaking) {
          strobe(Color.kBlue, strobeDuration, ledLeftLength, ledMiddleLenght);
        } else {
          solid(Color.kBlue, ledLeftLength, ledMiddleLenght);
        }

        if (leftReefSelected) {
          selektor(true, Color.kGreen);
        } else if (rightReefSelected) {
          selektor(true, Color.kGreen);
        }
        if (elevator_moving || climbing || autoDrive) {
          rainbow(rainbowCycleLength, rainbowDuration, 0, ledFullLength);
        } else if (demoMode) {
          wave(
              allianceColor,
              secondaryDisabledColor,
              waveAllianceCycleLength,
              waveAllianceDuration,
              0,
              ledFullLength);
        }

        if (endgameAlert) {
          strobe(Color.kRed, Color.kGold, strobeDuration, 0, ledFullLength);
        }
      }

      // Update LEDs
      leds.setData(buffer);
    }
  }

  private void solid(Color color, int startlength, int endLength) {
    if (color != null) {
      for (int i = startlength; i < endLength; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color, int startlength, int endLength) {
    for (int i = startlength; i < MathUtil.clamp(endLength * percent, 0, endLength); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration, int startlength, int endLength) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2, startlength, endLength);
  }

  private void strobe(Color color, double duration, int startlength, int endLenght) {
    strobe(color, Color.kBlack, duration, startlength, endLenght);
  }

  private void breath(Color c1, Color c2, int startLenght, int endLenght) {
    breath(c1, c2, Timer.getFPGATimestamp(), startLenght, endLenght);
  }

  private void breath(Color c1, Color c2, double timestamp, int startLenght, int endLenght) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue), startLenght, endLenght);
  }

  private void rainbow(double cycleLength, double duration, int startLenght, int endLenght) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = startLenght; i < endLenght; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(
      Color c1, Color c2, double cycleLength, double duration, int startlength, int endLenght) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = startlength; i < endLenght; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(
      List<Color> colors, int stripeLength, double duration, int startLenght, int endLenght) {
    int offset =
        (int) (Timer.getFPGATimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = startLenght; i < endLenght; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private void selektor(boolean sol, Color c1) {
    if (sol) {
      strobe(c1, breathDuration, 0, ledLeftLength);
    } else {
      strobe(c1, breathDuration, ledMiddleLenght, ledRightLength);
    }
  }
}
