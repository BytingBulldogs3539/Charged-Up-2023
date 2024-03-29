package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.ElevatorConstants;
import frc.robot.IDConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem.Arm;
import frc.robot.subsystems.ElevatorSubsystem.Wrist;

public class LEDSubsystem {
    boolean enabled;
    CANdle candle;

    public enum Piece {
        CONE,
        CUBE
    }
    Piece pickedUp = null;

    final double FLASH_SPEED = 0.25;
    final int NUM_LEDS = 56;
    final double MAX_BRIGHTNESS = 0.2;

    public enum LEDState {
        OFF,
        ON,
        READY,
        CONE,
        CUBE,
        FLASH_CONE,
        FLASH_CUBE,
        CLIMBING,
        DONE_CLIMBING
    }

    public LEDState state;
    private LEDState savedState;

    public LEDSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) return;

        this.candle = new CANdle(IDConstants.CandleID, IDConstants.CandleCanName);
		candle.configLEDType(LEDStripType.GRB);
		candle.configBrightnessScalar(MAX_BRIGHTNESS);

        setLEDs(LEDState.ON);
    }

    public void intake() {
        if (!enabled || pickedUp != null) return;

        if (RobotContainer.elevatorSubsystem.getWristOrientation() == Wrist.cone
              || RobotContainer.elevatorSubsystem.getArmLevel() == Arm.groundIntake) {
            setLEDs(LEDState.FLASH_CONE);
        } else {
            setLEDs(LEDState.FLASH_CUBE);
        }
    }

    public void extake() {
        if (!enabled) return;

        if (RobotContainer.elevatorSubsystem.getWristOrientation() == Wrist.cone
              || RobotContainer.elevatorSubsystem.getArmLevel() == Arm.groundIntake) {
            setLEDs(LEDState.FLASH_CONE);
            pickedUp = null;
        } else {
            setLEDs(LEDState.FLASH_CUBE);
            pickedUp = null;
        }
    }

    public void pieceTaken(Arm arm, Wrist wrist) {
        if (!enabled || pickedUp != null) return;

        if (wrist == Wrist.cone || arm == Arm.groundIntake) {
            setLEDs(LEDState.CONE);
            pickedUp = Piece.CONE;
        } else {
            setLEDs(LEDState.CUBE);
            pickedUp = Piece.CUBE;
        }
    }

    public void saveState() {
        if (!enabled) return;
        this.savedState = this.state;
    }
    
    public void restoreState() {
        if (!enabled || this.savedState == null) return;
        this.setLEDs(savedState);
    }

    public void setLEDs(LEDState state) {
        if (!enabled || this.state == state) return;
        this.state = state;

        switch (state) {
            case OFF:
                candle.animate(null);
                candle.setLEDs(0, 0, 0, 0, 0, ElevatorConstants.ledCount);
                break;
            case ON:
                candle.animate(null);
                candle.setLEDs(0, 255, 0, 0, 0, ElevatorConstants.ledCount);
                break;
            case READY:
                candle.animate(new ColorFlowAnimation(
                    0, 255, 0,
                    0, .5, NUM_LEDS, Direction.Forward
                ));
                break;
            case CONE:
                candle.animate(null);
                candle.setLEDs(255, 185, 0, 0, 0, ElevatorConstants.ledCount);
                break;
            case CUBE:
                candle.animate(null);
                candle.setLEDs(188, 0, 255, 0, 0, ElevatorConstants.ledCount);
                break;
            case FLASH_CONE:
                candle.animate(
                    new StrobeAnimation(255, 185, 0, 0, FLASH_SPEED, NUM_LEDS)
                );
                break;
            case FLASH_CUBE:
                candle.animate(
                    new StrobeAnimation(188, 0, 255, 0, FLASH_SPEED, NUM_LEDS)
                );
                break;
            case CLIMBING:
                candle.animate(
                    new StrobeAnimation(0, 0, 255, 0, FLASH_SPEED, NUM_LEDS)
                );
                break;
            case DONE_CLIMBING:
                candle.animate(
                    new RainbowAnimation(1, 0.5, NUM_LEDS)
                );
                break;
            default:
                System.out.println("\n\nLED Command State Error, State: " + state + "\n\n");
                break;
        }
    }
}
