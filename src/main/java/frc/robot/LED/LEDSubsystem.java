package frc.robot.LED;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Constants.*;

import org.frcteam3539.Byte_Swerve_Lib.Constants;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;

public class LEDSubsystem extends SubsystemBase {
	public static boolean enabled;

	public static boolean intaking;
	public static boolean aligning;
	public static boolean climbing;
	public static boolean shooting;
	public static boolean neutral;
	

	static CANdle candle;

	public LEDSubsystem(boolean enable) {
		enabled = enable;

		candle = new CANdle(IDConstants.CANdleID, "CANivore");
		
		CANdleConfiguration configs = new CANdleConfiguration();
		// Set the LED strip type and brightness
		configs.LED.StripType = StripTypeValue.GRB;
		configs.LED.BrightnessScalar = LEDConstants.maxBrightness;
		// Disable status LED when being controlled
		// configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
		// Write these configs to the CANdle
		candle.getConfigurator().apply(configs);

		setLEDs(LEDState.GREEN);
	}

	public enum LEDState {
		A, B, X, Y, POV_UP, POV_DOWN, POV_LEFT, POV_RIGHT,
		LEFT_TRIGGER, RIGHT_TRIGGER,
		ON, OFF, CONNECTED, AUTO, ERROR,
		RED, GREEN, BLUE, YELLOW, WHITE
	}

	private static LEDState state;

	// public static void setLEDS(Color color) {
	// 	candle.setControl(new SolidColor(0, LEDConstants.numLights)
	// 		.withColor(new RGBWColor((int) color.red, (int) color.green, (int) color.blue)));
	// }

	public static void setLEDs(LEDState state) {
		if (LEDSubsystem.state == state)
			return;

		// System.out.println("set state to " + state.toString());
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
		// Turns red if Estopped!!!
		if (DriverStation.isEStopped()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		if (!DriverStation.isEnabled()) {
			setLEDs(LEDState.GREEN);
			return;
		}
		// Turns Alliance Color when shooting 
		if( shooting && DriverStation.getAlliance().get() == Alliance.Blue){
			setLEDs(LEDState.BLUE);
			return;
		}
		if( shooting && DriverStation.getAlliance().get() == Alliance.Red){
			setLEDs(LEDState.RED);
			return;
		}
		if (ClimberSubsystem.getClimberPosition() > 20) {
			setLEDs(LEDState.GREEN);
			return;
		}
		// If we are climbing, it will gradually increase the LED bar with a rainbow loading bar.
		// if( ClimberPosition > 25 ){
		// 	setLEDs(LEDState.);
		// 	return;
		// }
		
		// if (aligning && !VisionSubsystem.frontCam.isConnected() && !VisionSubsystem.backCam.isConnected()) {
		// 	setLEDs(LEDState.ERROR);
		// 	return;
		// }

		// if (!OperatorTestController.testActive()) {
		// 	setLEDs(LEDState.OFF);
		// 	return;
		// }
		// if were doing nothing then LEDs turn white
		setLEDs(LEDState.WHITE);
	}
}
