package frc.robot.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import org.frcteam3539.Byte_Swerve_Lib.Constants;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;

public class LEDSubsystem extends SubsystemBase {

	static boolean intaking;
	static boolean enabled;
	static boolean aligning;
	static boolean climbing;
	static boolean reverseClimbing;

	static CANdle candle;

	public LEDSubsystem(boolean enable) {
		enabled = enable;

		candle = new CANdle(IDConstants.CANdleID, "CANivore");
		// candle.configLEDType(LEDStripType.GRB);
		// candle.configBrightnessScalar(LEDConstants.maxBrightness);
		setLEDs(LEDState.YELLOW);
	}

	public enum LEDState {
		A, B, X, Y, POV_UP, POV_DOWN, POV_LEFT, POV_RIGHT,
		LEFT_TRIGGER, RIGHT_TRIGGER,
		ON, OFF, CONNECTED, AUTO, ERROR,
		RED, GREEN, BLUE, YELLOW, WHITE
	}

	private static LEDState state;

	public static void setLEDS(Color color) {
		candle.setControl(new SolidColor(0, LEDConstants.numLights)
			.withColor(new RGBWColor((int) color.red, (int) color.green, (int) color.blue)));
	}

	public static void setLEDs(LEDState state) {
		if (LEDSubsystem.state == state)
			return;

		System.out.println("set state to " + state.toString());

		LEDSubsystem.state = state;

		switch (state) {
			case OFF:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(0, 0, 0)));
				break;

			case ON:
				candle.setControl(new ColorFlowAnimation(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue())));
				
				// candle.animate(new ColorFlowAnimation(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(),
				// 		LEDConstants.Green.getBlue(), 0, LEDConstants.flashSpeed, LEDConstants.numLights,
				// 		Direction.Forward));
				break;

			case CONNECTED:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue(), 255)));
				break;

			case B:
			case ERROR:
			case RED:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Red.getRed(), LEDConstants.Red.getGreen(), LEDConstants.Red.getBlue())));
				break;

			case A:
			case GREEN:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Green.getRed(), LEDConstants.Green.getGreen(), LEDConstants.Green.getBlue())));
				break;

			case X:
			case BLUE:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(), LEDConstants.Blue.getBlue())));
				break;

			case Y:
			case YELLOW:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Yellow.getRed(), LEDConstants.Yellow.getGreen(), LEDConstants.Yellow.getBlue())));
				break;

			case WHITE:
				candle.setControl(new SolidColor(0, LEDConstants.numLights)
					.withColor(new RGBWColor(255, 255, 255)));
				break;

			case AUTO:
				candle.setControl(new StrobeAnimation(0, LEDConstants.numLights)
					.withColor(new RGBWColor(LEDConstants.Blue.getRed(), LEDConstants.Blue.getGreen(), LEDConstants.Blue.getBlue())));
				break;

			default:
				break;
		}
	}

	@Override
	public void periodic() {
		if (DriverStation.isEStopped()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		// if (aligning && !VisionSubsystem.frontCam.isConnected() && !VisionSubsystem.backCam.isConnected()) {
		// 	setLEDs(LEDState.ERROR);
		// 	return;
		// }

		// if (!OperatorTestController.testActive()) {
		// 	setLEDs(LEDState.OFF);
		// 	return;
		// }

		setLEDs(LEDState.CONNECTED);
	}
}
