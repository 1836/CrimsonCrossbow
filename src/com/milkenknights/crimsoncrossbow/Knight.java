/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.milkenknights.crimsoncrossbow;

import com.milkenknights.crimsoncrossbow.InsightLT.DecimalData;
import com.milkenknights.crimsoncrossbow.InsightLT.InsightLT;
import com.milkenknights.crimsoncrossbow.InsightLT.StringData;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SafePWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.fpga.tSPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Knight extends IterativeRobot {

        // Constants...
	private static final double JITTER_RANGE = 0.008;

	// For slow mode
	private static final double SLOW_MOD = 0.6;

	// For bang bang mode
	public static final double SHOOTER_RPM_HIGH = 3700;
	public static final double SHOOTER_RPM_LOW = 3400;

	// For voltage mode
	public static final double SHOOTER_POWER_HIGH = 0.7;
	public static final double SHOOTER_POWER_LOW = 0.6;

	// For Arduino I2C
	private static final int ARDUINO_I2C_ADDRESS = 0x3A;

	// Pseudo-enumerator for Auton options...
	public static int AUTON_CENTER = 0;
	public static int AUTON_SIDE = 1;

	JStick xbox; // XBox controller
	JStick atk; // Logitech ATK3 controller
	JStick autonStick; // the joystick used for testing PID

	private boolean usingCheesy;
	DriverStationLCD lcd;

	private Compressor compressor;

	// Pair state "true" means high gear,
	// Pair state "false" means low gear
	public SolenoidPair driveGear;

	// used to remember the gear that is being used
	// while in and out of slow mode
	private boolean normalGear;

	private SolenoidPair hookClimb;
	private SolenoidPair caster;

	public Drive drive;
	private SpeedController leftWheel;
	private SpeedController rightWheel;
	private SpeedController shooter;
	private SpeedController actuator;
	private SpeedController kicker;

	private static final int SHOOTER_MODE_VOLTAGE = 0;
	private static final int SHOOTER_BANG_BANG = 1;
	private static final int SHOOTER_PID = 2;
	private static final int SHOOTER_COMBINED = 3;
	private int shooterMode;

	private final int I2C_DELAY = 1000;
	private int countMe = 0;

	private boolean sentIdle;

	private Counter shooterEnc;
	private Counter kickerEnc;
	
	private tSPI spi;

	public Encoder leftEnc;
	public Encoder rightEnc;

	private Gyro gyro;
	private PIDController gyroPID;

	private boolean lightIsOn;
	private Relay light;

	// Setup auton classes....
	private Auton centerAuton;
	private Auton sideAuton;

	public double autonStart;

	Command autonomousCommand;
	SendableChooser autoChooser;

	// Used to determine which autonomous procedure to use
	private DigitalInput autonCheck;

	// stuff for the InsightLT display
	private InsightLT display;
	private DecimalData disp_batteryVoltage;
	private StringData disp_message;

	PIDController leftWheelPID;
	PIDController rightWheelPID;
	
	Vision vision;

	private void defaultVoltageShooter(boolean on) {
		voltageShooter(on, 0.65);
	}

	private void voltageShooter(boolean on, double frac) {
		double output = on ? Utils.voltageSpeed(frac) : 0;
		shooter.set(output);
		kicker.set(output);
	}

	public void bangBangShooter(boolean on, double targetRPM) {
		double shooterOutput;
		if (on) {
			shooterOutput = Utils.getBangBang(targetRPM, 0.6, shooterEnc);
		} else {
			shooterOutput = 0;
		}
		shooter.set(shooterOutput);
		kicker.set(shooterOutput);
	}

	private void combinedShooter(boolean on, double frac, double targetRPM) {
		if (on) {
			// shooter gets bang bang at the low speed
			// kicker gets 80% voltage
			shooter.set(Utils.getBangBang(targetRPM,0.6,shooterEnc));
			kicker.set(Utils.voltageSpeed(frac));
		} else {
			shooter.set(0);
			kicker.set(0);
		}
	}

	private void shooterOff() {
		shooter.set(0);
		kicker.set(0);
	}

	public void defaultActuator(boolean on) {
		actuator.set(on ? 0.4 : 0);
	}

	// Resets encoders in one easy function!
	private void resetEncoders() {
		rightEnc.reset();
		leftEnc.reset();
	}

	public Knight() {
		shooterMode = SHOOTER_BANG_BANG;

		xbox = new JStick(1);
		xbox.setSlow(0.3);

		atk = new JStick(2);
		
		autonStick = new JStick(3);

		lcd = DriverStationLCD.getInstance();

		usingCheesy = true;
		integral_err = 0;
		prev_err = 0;

		vision = new Vision();
		
		//autonCheck = new DigitalInput(AUTON_CHECK_DI);

		// configure the display to have two lines of text
		display = new InsightLT(InsightLT.TWO_ONE_LINE_ZONES);
		display.startDisplay();

		// add battery display
		disp_batteryVoltage = new DecimalData("Bat:");
		display.registerData(disp_batteryVoltage,1);

		// this shows what mode the robot is in
		// i.e. teleop, autonomous, disabled
		disp_message = new StringData();
		display.registerData(disp_message,2);


	}
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

		// Setup arduino comm
		spi = new tSPI();
		
		
		ConfigFile robotConfig = new ConfigFile("robot-config.txt");
		robotConfig.loadFile();
		//SmartDashboard.putString("hello",robotConfig.get("hello"));

		leftWheel = new Talon(robotConfig.getAsInt("tLeftWheel"));
		rightWheel = new ReversableTalon(robotConfig.getAsInt("tRightWheel"),true);
		System.out.println("hello hello");
		drive = new Drive(leftWheel,rightWheel);

		shooter = new Talon(robotConfig.getAsInt("tShooter"));
		actuator = new Talon(robotConfig.getAsInt("tActuator"));
		kicker = new Talon(robotConfig.getAsInt("tKicker"));

		compressor = new Compressor(robotConfig.getAsInt("compressorPressureSwitch"),
				robotConfig.getAsInt("compressorRelayChannel"));
		driveGear = new SolenoidXORPair(robotConfig.getAsInt("sDriveGearA"),
				robotConfig.getAsInt("sDriveGearB"));
		normalGear = driveGear.get();
		hookClimb = new SolenoidXANDPair(robotConfig.getAsInt("sHookA"),
				robotConfig.getAsInt("sHookB"));
		caster = new SolenoidXANDPair(robotConfig.getAsInt("sCasterA"),
					robotConfig.getAsInt("sCasterB"));

		kickerEnc = new Counter(robotConfig.getAsInt("kickerEnc"));
		shooterEnc = new Counter(robotConfig.getAsInt("shooterEnc"));
		leftEnc = new Encoder(robotConfig.getAsInt("leftEncA"),
			   robotConfig.getAsInt("leftEncB"), true, EncodingType.k4X);
		rightEnc = new Encoder(robotConfig.getAsInt("rightEncA"),
				robotConfig.getAsInt("rightEncB"), false, EncodingType.k4X);
		// inches
		leftEnc.setDistancePerPulse(robotConfig.getAsDouble("leftEncPulse"));
		rightEnc.setDistancePerPulse(robotConfig.getAsDouble("rightEncPulse"));

		gyro = new Gyro(robotConfig.getAsInt("gyro"));
		gyro.setPIDSourceParameter(PIDSource.PIDSourceParameter.kAngle);

		light = new Relay(robotConfig.getAsInt("lightRelay"));
		compressor.start();
		driveGear.set(true);

		drive.setInvertedMotor(RobotDrive.MotorType.kRearRight,true);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft,true);

		kickerEnc.start();
		shooterEnc.start();

		leftEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		rightEnc.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);

		leftWheelPID = new PIDController(
				robotConfig.getAsDouble("l_kp"),
				robotConfig.getAsDouble("l_ki"),
				robotConfig.getAsDouble("l_kd"),
				robotConfig.getAsDouble("l_kf"),
				leftEnc, leftWheel);

		rightWheelPID = new PIDController(
				robotConfig.getAsDouble("r_kp"),
				robotConfig.getAsDouble("r_ki"),
				robotConfig.getAsDouble("r_kd"),
				robotConfig.getAsDouble("r_kf"),
				rightEnc, rightWheel);
		/*
		class OppositeTalons implements PIDOutput {
			private Talon a;
			private Talon b;
			
			public OppositeTalons(Talon a, Talon b) {
				this.a = a;
				this.b = b;
			}
			public void pidWrite(double output) {
				a.set(output);
				b.set(-output);
			}
		}
		*/
		gyroPID = new PIDController(0.01,0,0,0,gyro,new PIDOutput() {
			public void pidWrite(double output) {
				leftWheel.set(output);
				rightWheel.set(-output);
			}
		});
		
		leftWheelPID.startLiveWindowMode();
		rightWheelPID.startLiveWindowMode();
		gyroPID.startLiveWindowMode();

		SmartDashboard.putData(Scheduler.getInstance());

		// Setup Autonomous chooser
		centerAuton = new Auton(this, AUTON_CENTER);
		sideAuton = new Auton(this, AUTON_SIDE);

		autoChooser = new SendableChooser();
		autoChooser.addDefault("Center Autonomous", centerAuton);
		autoChooser.addObject("Side Autonomous", sideAuton);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

	}

    //This function is called at the start of autonomous
	Timer timer;

	int frisbeesThrown;
	public void autonomousInit() {
		autonStart = Timer.getFPGATimestamp();
		leftEnc.reset();
		rightEnc.reset();

		leftWheelPID.setSetpoint(100);
		rightWheelPID.setSetpoint(100);

		leftWheelPID.enable();
		rightWheelPID.enable();

		leftEnc.start();
		rightEnc.start();

		/*
		autonomousCommand = (Command) autoChooser.getSelected();
		autonomousCommand.start();
		*/
	}
	/**
	 * This function is called periodically during autonomous
	 */
	double integral_err;
	double prev_err;
	double last_timer;
	boolean frisbeeDone;
	final double WAIT_AFTER_ACTUATOR = 1;
	final double WAIT_AFTER_SHOOTING = WAIT_AFTER_ACTUATOR+3.5;
	final double DELAY_BETWEEN_FRISBEES = 2.25;
	final double FRISBEE_SHOOT_TIME = 0.25;
	final double DRIVE_DISTANCE = 102;
	final double SLOWDOWN_TIME = 0.25;

	final double DRIVE_FORWARD_TIME = 2;

	public double finishedMovingForward = -1;

	public void autonomousPeriodic() {
		SmartDashboard.putBoolean("left pid is on",leftWheelPID.isEnable());
		SmartDashboard.putNumber("left p",leftWheelPID.getP());
		SmartDashboard.putNumber("left i",leftWheelPID.getI());
		SmartDashboard.putNumber("left d",leftWheelPID.getD());
		SmartDashboard.putNumber("left f",leftWheelPID.getF());
		SmartDashboard.putNumber("left setpoint",leftWheelPID.getSetpoint());
		SmartDashboard.putNumber("left error",leftWheelPID.getError());
		SmartDashboard.putNumber("Left Distance", leftEnc.getDistance());
		SmartDashboard.putNumber("Left Distance", leftEnc.getDistance());
		SmartDashboard.putNumber("Left Raw", leftEnc.getRaw());


		SmartDashboard.putBoolean("right pid is on",rightWheelPID.isEnable());
		SmartDashboard.putNumber("right p",rightWheelPID.getP());
		SmartDashboard.putNumber("right i",rightWheelPID.getI());
		SmartDashboard.putNumber("right d",rightWheelPID.getD());
		SmartDashboard.putNumber("right f",rightWheelPID.getF());
		SmartDashboard.putNumber("right setpoint",rightWheelPID.getSetpoint());
		SmartDashboard.putNumber("right error",rightWheelPID.getError());
		SmartDashboard.putNumber("Right Distance", rightEnc.getDistance());
		SmartDashboard.putNumber("Right Raw", rightEnc.getRaw());
		
		SmartDashboard.putNumber("Right Wheels", rightWheel.get());
		SmartDashboard.putNumber("Left Wheels", leftWheel.get());
	}
	public void teleopInit() {
		//light.set(Relay.Value.kForward);
		leftEnc.start();
		rightEnc.start();
	}
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

		xbox.update();
		atk.update();
		autonStick.update();
		
		if (autonStick.isReleased(3)) {
			leftWheelPID.setSetpoint(24);
			leftWheelPID.enable();
			rightWheelPID.setSetpoint(24);
			rightWheelPID.enable();
		}
		if (autonStick.isReleased(2)) {
			leftWheelPID.disable();
			rightWheelPID.disable();
			gyroPID.disable();
		}
		if (autonStick.isReleased(4)) {
			leftWheelPID.setSetpoint(0);
			leftWheelPID.enable();
			rightWheelPID.setSetpoint(0);
			rightWheelPID.enable();
		}
		if (autonStick.isReleased(5)) {
			leftWheelPID.setSetpoint(100);
			leftWheelPID.enable();
			rightWheelPID.setSetpoint(100);
			rightWheelPID.enable();
		}
		if (autonStick.isReleased(1)) {
			leftWheelPID.disable();
			leftWheelPID.reset();
			rightWheelPID.disable();
			rightWheelPID.reset();
			gyroPID.disable();
			gyroPID.reset();
		}
		if (autonStick.isReleased(8)) {
			gyro.reset();
		}
		if (autonStick.isReleased(9)) {
			gyroPID.setSetpoint(0);
			gyroPID.enable();
		}
		
		if (autonStick.isReleased(10)) {
			SmartDashboard.putNumber("Target found", vision.isHot());
		}
		
		
		SmartDashboard.putBoolean("left pid is on",leftWheelPID.isEnable());
		SmartDashboard.putNumber("left p",leftWheelPID.getP());
		SmartDashboard.putNumber("left i",leftWheelPID.getI());
		SmartDashboard.putNumber("left d",leftWheelPID.getD());
		SmartDashboard.putNumber("left f",leftWheelPID.getF());
		SmartDashboard.putNumber("left setpoint",leftWheelPID.getSetpoint());
		
		SmartDashboard.putBoolean("right pid is on",leftWheelPID.isEnable());
		SmartDashboard.putNumber("right p",rightWheelPID.getP());
		SmartDashboard.putNumber("right i",rightWheelPID.getI());
		SmartDashboard.putNumber("right d",rightWheelPID.getD());
		SmartDashboard.putNumber("right f",rightWheelPID.getF());
		SmartDashboard.putNumber("right setpoint",rightWheelPID.getSetpoint());
		
		SmartDashboard.putNumber("Right Wheels", rightWheel.get());
		SmartDashboard.putNumber("Left Wheels", leftWheel.get());
		
		// only graph error when PID is on
		if (leftWheelPID.isEnable()) {
			SmartDashboard.putNumber("left error",leftWheelPID.getError());
		}
		if (rightWheelPID.isEnable()) {
			SmartDashboard.putNumber("right error",rightWheelPID.getError());
		}
		if (gyroPID.isEnable()) {
			//SmartDashboard.putNumber("gyro error",gyroPID.getError());
		}
		SmartDashboard.putNumber("gyro error",gyroPID.getError());


		// Press A to toggle cheesy drive
		if (xbox.isReleased(JStick.XBOX_A)) {
			usingCheesy = !usingCheesy;
		}

		// use LB to toggle high and low gear
		if (xbox.isReleased(JStick.XBOX_LB)) {
			driveGear.toggle();
			normalGear = !normalGear;
		}

		// show the solenoids status
		lcd.println(DriverStationLCD.Line.kUser3,1,driveGear.get()?"High Gear":"Low Gear ");

		// show if the compressor is running
		if (compressor.getPressureSwitchValue()) {
			lcd.println(DriverStationLCD.Line.kUser6,1,"Compressor is off    ");
		} else {
			lcd.println(DriverStationLCD.Line.kUser6,1,"Compressor is running");
		}

		// joystick button 1 spins the actuator
		defaultActuator(atk.isPressed(1));

		/*
		// change shooter modes
		if (atk.isPressed(11)) {
			shooterMode = SHOOTER_MODE_VOLTAGE;
		} else if (atk.isPressed(10)) {
			shooterMode = SHOOTER_BANG_BANG;
		} else if (atk.isPressed(9)) {
			shooterMode = SHOOTER_COMBINED;
		}
		*/

		if (shooterMode == SHOOTER_MODE_VOLTAGE) {
			if (atk.isPressed(2)) {
				defaultVoltageShooter(true);
			} else if (atk.isPressed(4) || atk.isPressed(5)) {
				voltageShooter(true, 0.6);
			} else {
				shooterOff();
			}
			//defaultVoltageShooter(atk.isPressed(2));
		} else if (shooterMode == SHOOTER_BANG_BANG) {
			if (atk.isPressed(2)) {
				bangBangShooter(true, SHOOTER_RPM_HIGH);
			} else if (atk.isPressed(4) || atk.isPressed(5)) {
				bangBangShooter(true, SHOOTER_RPM_LOW);
			} else {
				shooterOff();
			}
		} else if (shooterMode == SHOOTER_PID) {
			// TO: shooter PID
		} else if (shooterMode == SHOOTER_COMBINED) {
			if (atk.isPressed(2)) {
				combinedShooter(true,SHOOTER_POWER_HIGH,SHOOTER_RPM_HIGH);
			} else if (atk.isPressed(4) || atk.isPressed(5)) {
				combinedShooter(true, SHOOTER_POWER_LOW,SHOOTER_RPM_LOW);
			} else {
				shooterOff();
			}
		} else {
			shooterOff();
		}

		// toggle the hook climb
		if (atk.isReleased(11)) {
			hookClimb.toggle();
		}

		// toggle the caster
		if (xbox.isReleased(JStick.XBOX_RB)) {
			caster.toggle();
		}
		
		// toggle the light
		if (xbox.isReleased(JStick.XBOX_Y)) {
			lightIsOn = !lightIsOn;
			if (lightIsOn) {
				light.set(Relay.Value.kForward);
			} else {
				light.set(Relay.Value.kOff);
			}
		}

		//double leftStickX = JStick.removeJitter(xbox.getAxis(JStick.XBOX_LSX), JITTER_RANGE);
		double leftStickY = JStick.removeJitter(xbox.getAxis(JStick.XBOX_LSY), JITTER_RANGE);
		double rightStickX = JStick.removeJitter(xbox.getAxis(JStick.XBOX_RSX), JITTER_RANGE);
		double rightStickY = JStick.removeJitter(xbox.getAxis(JStick.XBOX_RSY), JITTER_RANGE);


		boolean slowMode = xbox.getAxis(JStick.XBOX_TRIG) < -0.5;
		if (slowMode) {
			//driveGear.set(false);
		} else {
			//driveGear.set(normalGear);
		}

		if (usingCheesy) {
			drive.cheesyDrive(xbox.getSlowedAxis(JStick.XBOX_LSY)*(slowMode?SLOW_MOD:1), rightStickX,
					//xbox.isPressed(JStick.XBOX_LJ)
					// If either trigger is pressed, enable quickturn
					Math.abs(xbox.getAxis(JStick.XBOX_TRIG)) > 0.5
					);
			lcd.println(DriverStationLCD.Line.kUser4,1,"cheesy drive");
		} else {
			drive.tankDrive(leftStickY*(slowMode?SLOW_MOD:1), rightStickY*(slowMode?SLOW_MOD:1));
			lcd.println(DriverStationLCD.Line.kUser4,1,"tank drive   ");
		}

		if (shooterMode == SHOOTER_MODE_VOLTAGE) {
			lcd.println(DriverStationLCD.Line.kUser1,1,"Shooter mode:voltage ");
		} else if (shooterMode == SHOOTER_BANG_BANG) {
			lcd.println(DriverStationLCD.Line.kUser1,1,"Shooter mode:bangbang");
		} else if (shooterMode == SHOOTER_COMBINED) {
			lcd.println(DriverStationLCD.Line.kUser1,1,"Shooter mode:combined");
		} else {
			lcd.println(DriverStationLCD.Line.kUser1,1,"Shooter mode:????????");
		}

		// Send shooter speeds to arduino
		//arduinoComm.write(ARDUINO_I2C_ADDRESS, (int) (60/shooterEnc.getPeriod())/100);

		// print encoder values to see if they're working
		lcd.println(DriverStationLCD.Line.kUser2,1,""+shooterEnc.getPeriod());

		SmartDashboard.putNumber("Shooter speed", shooterEnc.getPeriod());
		SmartDashboard.putNumber("Shooter RPM", 60/shooterEnc.getPeriod());
		SmartDashboard.putNumber("Shooter count", shooterEnc.get());
		SmartDashboard.putNumber("Kicker speed", kickerEnc.getPeriod());
		SmartDashboard.putNumber("Kicker RPM", 60/kickerEnc.getPeriod());
		SmartDashboard.putNumber("Kicker count", kickerEnc.getPeriod());
		SmartDashboard.putNumber("Left Rate", leftEnc.getRate());
		SmartDashboard.putNumber("Left Distance", leftEnc.getDistance());
		SmartDashboard.putNumber("Left Raw", leftEnc.getRaw());
		SmartDashboard.putNumber("Right Rate", rightEnc.getRate());
		SmartDashboard.putNumber("Right Distance", rightEnc.getDistance());
		SmartDashboard.putNumber("Right Raw", rightEnc.getRaw());
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		//SmartDashboard.putBoolean("Auton check", autonCheck.get());

		SmartDashboard.putNumber("Right Wheels", rightWheel.get());
		SmartDashboard.putNumber("Left Wheels", leftWheel.get());

		lcd.updateLCD();

		// update the display
		disp_batteryVoltage.setData(DriverStation.getInstance().getBatteryVoltage());
		disp_message.setData("teleop");

		if (countMe > I2C_DELAY) {

			System.out.println("What I'm sending" + (60 / shooterEnc.getPeriod()) / 100);
			System.out.println("What it is." + shooterEnc.getPeriod());

			countMe = 0;
		} else {
			countMe++;
		}
		
		// pid stuff in the window
		leftWheelPID.updateTable();
		
		SmartDashboard.putData("left wheel pid",leftWheelPID);
		SmartDashboard.putData("right wheel pid",rightWheelPID);
		SmartDashboard.putData("gyro pid",gyroPID);
		

    }

	public void disabledPeriodic() {
		//disp_batteryVoltage.setData(DriverStation.getInstance().getBatteryVoltage());
		//disp_message.setData("disabled");

		if (countMe > I2C_DELAY) {
			countMe++;
		} else {
			countMe = 0;
		}

		//$wag
		leftEnc.reset();
		rightEnc.reset();
		
		leftWheelPID.disable();
		rightWheelPID.disable();

		/*
		if (DriverStation.getInstance().isFMSAttached()) {
    		display.stopDisplay();
    	} else {
    		display.startDisplay();
    	}
    	*/
	}

	private boolean shootTester;

	private boolean pwmtest;
	private SafePWM[] pwms;

	public void testInit() {
		timer.start();
		pwmtest = false;
		pwms = new SafePWM[10];
		for (int i = 0; i < 10; ++i) {
			pwms[i] = new SafePWM(i+1);
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		/*
		xbox.update();
		atk.update();

		// toggle between PWM test and austin's thing
		if (xbox.isPressed(JStick.XBOX_LB)) {
			pwmtest = false;
		}
		if (xbox.isPressed(JStick.XBOX_RB)) {
			pwmtest = true;
		}

		if (pwmtest) {
			for (int i = 0; i < 10; ++i) {
				if (atk.isPressed(i+1)) {
					pwms[i].setRaw(143);
				}
			}
		} else {
			if (xbox.isReleased(JStick.XBOX_A)) {
					shootTester = !shootTester;
					lcd.println(DriverStationLCD.Line.kUser1, 1, "Shooter Tester     ");
				} else {
					lcd.println(DriverStationLCD.Line.kUser1, 1, "Normal Tester      ");
			}

			//Only spins shooter
			shooter.set((atk.isPressed(7)) ? -1 : 0);
			//Only spins the kicker
			kicker.set((atk.isPressed(6)) ? 1 : 0);
			//Slow start for shooting 1
			if(shootTester && atk.isPressed(1)) {
				if(timer.get() > 2) {
					shooter.set(1);
					lcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter: On    ");
				}
				if(timer.get() > 4) {
					kicker.set(1);
					lcd.println(DriverStationLCD.Line.kUser3, 1, "Kicker: On     ");
				}
				if(timer.get() > 7) {
					//actuator.set(1);
					lcd.println(DriverStationLCD.Line.kUser4, 1, "CAM: On        ");
				} else {
					timer.reset();
					shooter.set(0);
					kicker.set(0);
					actuator.set(0);
					lcd.println(DriverStationLCD.Line.kUser2, 1, "Shooter: Off   ");
					lcd.println(DriverStationLCD.Line.kUser3, 1, "Kicker: Off    ");
					lcd.println(DriverStationLCD.Line.kUser4, 1, "CAM: Off       ");
				}
			}
			lcd.println(DriverStationLCD.Line.kUser1, 1, "" + timer.get());
			lcd.updateLCD();
		}
		*/
	}
}
