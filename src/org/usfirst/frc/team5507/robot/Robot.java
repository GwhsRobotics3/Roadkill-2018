/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5507.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team5507.grip.CubePipeline;
import org.usfirst.frc.team5507.robot.commands.ExampleCommand;
import org.usfirst.frc.team5507.robot.subsystems.ExampleSubsystem;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static final ExampleSubsystem kExampleSubsystem
	= new ExampleSubsystem();
	public static OI m_oi;
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	private MecanumDrive drive;
	//Objects that control the motors
	private WPI_TalonSRX fLeft = new WPI_TalonSRX(RobotMap.frontLeft); 
	private WPI_TalonSRX fRight = new WPI_TalonSRX(RobotMap.frontRight); 
	private WPI_TalonSRX rLeft = new WPI_TalonSRX(RobotMap.rearLeft); 
	private WPI_TalonSRX rRight = new WPI_TalonSRX(RobotMap.rearRight); 
	private Joystick m_joystick = new Joystick(RobotMap.JoystickPort); //Object to set XBox controller buttons to control different motors
	int kTimeoutMs = 10;
	double targetPos = 0;
	boolean isRPressed = false;
	boolean isLPressed = false;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		m_chooser.addDefault("Default Auto", new ExampleCommand());

		drive = new MecanumDrive(fLeft, rLeft, fRight, rRight); //For driving robot

		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		fLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		fLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		fLeft.setSensorPhase(true);
		fLeft.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate*/
		fLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		fLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* set the peak and nominal outputs, 12V means full */
		fLeft.configNominalOutputForward(0, kTimeoutMs);
		fLeft.configNominalOutputReverse(0, kTimeoutMs);
		fLeft.configPeakOutputForward(1, kTimeoutMs);
		fLeft.configPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		fLeft.selectProfileSlot(0, 0);
		fLeft.config_kF(0, 0.2, kTimeoutMs);
		fLeft.config_kP(0, 0.2, kTimeoutMs);
		fLeft.config_kI(0, 0, kTimeoutMs);
		fLeft.config_kD(0, 0, kTimeoutMs);
		/* set acceleration and vcruise velocity - see documentation */
		fLeft.configMotionCruiseVelocity(15000, kTimeoutMs);
		fLeft.configMotionAcceleration(6000, kTimeoutMs);
		/* zero the sensor */
		fLeft.setSelectedSensorPosition(0, 0, kTimeoutMs);
		//cameraInit();
	}

	public void cameraInit() {
		//Vision processing to recognize cube
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
		visionThread = new VisionThread(camera, new CubePipeline(), pipeline -> {
			if (!pipeline.filterContoursOutput().isEmpty()) {
				Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
				}
			}
		});
		visionThread.start();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		double turn = centerX - (RobotMap.IMG_WIDTH / 2);
		drive.driveCartesian(0.05, 0.05, turn);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		drive.driveCartesian(-m_joystick.getRawAxis(0), //Forward Back Axis
							 m_joystick.getRawAxis(1), //Left Right Axis
							 m_joystick.getRawAxis(3) - m_joystick.getRawAxis(2)); //Rotation with shoulder bumpers
		// Have right (6) bumper add 8 and left (5) bumper -8 from targetpos. Only when button is pressed.
//		if (m_joystick.getRawButton(1)) {
//
//			/* Motion Magic */
//			//double targetPos = m_joystick.getRawAxis(5) * 4096 * 0.5; /* 4096 ticks/rev * 10 Rotations in either direction */
//			k_motor.set(ControlMode.MotionMagic, targetPos);
//		} else {
//			if (Math.abs(m_joystick.getRawAxis(5)) > 0.1)
//			{
//				k_motor.set(ControlMode.PercentOutput, m_joystick.getRawAxis(5));
//			}
//			else
//			{
//				k_motor.set(ControlMode.PercentOutput, 0);
//			}
//		}

		if (m_joystick.getRawButton(5))
		{
			if (!isLPressed)
			{
				targetPos -= 128;
				isLPressed = true;	
			}
		}
		else 
		{
			isLPressed = false;
		}

		if (m_joystick.getRawButton(6))
		{
			if (!isRPressed)
			{
				targetPos += 128;
				isRPressed = true;
			}
		}
		else 
		{
			isRPressed = false;
		}


		double speed = fLeft.getSelectedSensorVelocity(0);
		double position = fLeft.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("spd", speed);
		SmartDashboard.putNumber("pos", position);
		SmartDashboard.putNumber("targetPos", targetPos);

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
