#include "ahrs.h"
#include "pixy.h"
#include "Profile.h"
#include "AltMotor.h"
#include "wpilib.h"

using namespace frc;

/*
 * 2017 Robot for Team 6055 - Steamworks
 *
 *PWM 0		Left Side Motors
 *PWM 1		Right Side Motors
 *PWM 2		Ball Intake Motor
 *PWM 3		Shooter Motor
 *PWM 4		SPARE
 *PWM 5		Winch
 *PWM 6		Ball Feed Servo
 *PWM 7     Gear Feed Motor
 *DIO 0,1  	SPARE
 *DIO 2,3  	Encoder Right Side
 *DIO 4     Gear Feed Home position
 *AIO 0     SPARE
 *AIO 1     Gear Feed Motor Encoder (used in AltMotor.cpp)
 *AIO 2     SPARE
 *Pixy1   	Targeting camera, Onboard I2C on RoborRIO, Address 0x54, custom cable
 *Camera	Axis visual camera, ethernet to radio, smartdashboard url = http://10.60.55.11/mjpg/video.mjpg  (default port = 1180 may use 5800?)
 *NavX		MXP port on RoboRIO
 *
 *
 *TankDrive			Two joysticks
 *Shooter Motor		Right joystick trigger
 *Shoot	    		Right joystick button 3
 *Shoot	Speed       (in Manual or Auto) Right joystick throttle
 *Intake			Left joystick trigger
 *Winch IN			Left joystick button 3
 *
 *Drive Direction	Left joystick throttle             up for forward - down for reverse
 *Gear Feed			Left or right joystick button 11 - push for feed position, let go for home position
 *													   ignores any push or release while moving
 *Gear Release		Left or right joystick button 12 - push for release position, let go for home position
 *													   ignores any push or release while moving
 *
 *Version 1.0  - 1/20/2017 - CRM  baseline program - untested
 *Version 1.1  - 1/29/2017 - CRM  added shooter speed control
 *Version 1.2  - 2/02/2017 - CRM  changed navx protocol to I2C from SPI
 *								  added GetHeading function to normalize yaw from navx
 *								  added ZeroHeading function to provide offset
 *								  corrected steering error in Profile.cpp
 *								  corrected motor directions
 *								  corrected agitator speed when trigger not pulled
 *Version 1.3  - 2/03/2017 - CRM  added library for bosch seat motor with built-in encoder
 *								  added GearFeed motor on either joystick button 10
 *								  tested servo cycle for shooter
 *								  tested code for seat position motor
 *Version 1.4  - 2/06/2017 - CRM  hard-coded servo positions and removed from preferences
 *								  added GearRelease on either joystick button 11
 *								  added autonomous shooting profiles
 *								  tested GearFeed and GearRelease functions on robot
 *Version 1.5  - 2/07/2017 - CRM  changed buttons for drive direction and gearfeed and gearrelease
 *								  added code for Gear Home position limit switch
 *								  changed gear release position to 90 deg
 *Version 1.6  - 2/16/2017 - CRM  added analog input for pixy as alternate method
 *								  removed sleep call in pixy I2C code
 *Version 1.7  - 2/21/2017 - CRM  reverse drive direction was looking at right stick instead of left
 *								  remove agitator motor
 *Version 1.8  - 2/27/2017 - CRM  made autonomous steer gains follow motor direction
 *								  pixy steer gain is fixed - does not follow motor direction
 *								  successfully tested vertical operation of navx
 *								  corrected problem with display of steer reversed
 *								  successfull test of autonomous gear hang
 *								  set navx back to SPI - still have crc errors but works fine
 *Version 1.9  - 2/28/2017 - CRM  successfull test of autonomous shooting
 *							      corrected problem with spinupdelay in autonomous
 *Version 1.91 - 3/01/2017 - CRM  fix problem with steering direction
 *Version 1.92 - 3/06/2017 - CRM  change distances and angles for autonomous
 *								  remove code for timed cycling servo for ball shooting - now just in or out on button 3 of right stick
 *Version 1.93 - 3/07/2017 - CRM  change distances and angles for autonomous again after testing
 *                                slow down right before gear hang
 *                                rewrote gear hanger code to be state machine - added timeouts in both directions
 *				=				  add option to shoot after gear hang from position 1 and 3/.,
 *Version 1.94 - 3/08/2017 - CRM  add options for passive gear hang
 *                                make slow down for gear hang 12" instead of 6"
 */

class Robot: public IterativeRobot
{
private:
	LiveWindow* LW = LiveWindow::GetInstance();
	VictorSP MotorLeftSide, MotorRightSide;
	VictorSP MotorIntake, MotorShooter, MotorWinch;
	Servo ServoBallFeed;
	AltMotor MotorGearFeed;
	RobotDrive DriveTrain;
	Encoder LeftEncoder,RightEncoder;
	Joystick StickLeft, StickRight;
	PowerDistributionPanel PDP;
	AnalogInput Sonar;
	DigitalInput GearHomeSwitch;
	Pixy PixyCam;

	SendableChooser<std::string> Chooser;
	const std::string AutoDoNothing= "AutoDoNothing";
	const std::string AutoCrossBaseLine1 = "AutoCrossBaseLine1";
	const std::string AutoCrossBaseLine2 = "AutoCrossBaseLine2";
	const std::string AutoCrossBaseLine3 = "AutoCrossBaseLine3";
	const std::string AutoHangGear1 = "AutoHangGear1";
	const std::string AutoHangGear1Passive = "AutoHangGear1Passive";
	const std::string AutoHangGear1AndShoot = "AutoHangGear1AndShoot";
	const std::string AutoHangGear2 = "AutoHangGear2";
	const std::string AutoHangGear2Passive = "AutoHangGear2Passive";
	const std::string AutoHangGear3 = "AutoHangGear3";
	const std::string AutoHangGear3Passive = "AutoHangGear3Passive";
	const std::string AutoHangGear3AndShoot = "AutoHangGear3AndShoot";
	const std::string AutoShoot1 = "AutoShoot1";
	const std::string AutoShoot3 = "AutoShoot3";
	std::string AutoSelected;

	SendableChooser<std::string> Chooser2;
	const std::string ShooterSpeedManual = "ShooterSpeedManual";
	const std::string ShooterSpeedAuto = "ShooterSpeedAuto";
	const std::string ShooterSpeedFixed = "ShooterSpeedFixed";
	std::string ShooterSpeedSelected;

	Preferences *Prefs;
	float ShooterSpeed = -1.0f;    		// speed of motor for shooting ball
	uint64_t ShooterSpinupDelay = 500; 	// delay for motor spinup
	float IntakeSpeed = -0.50f;    		// speed of motor for intake of ball
	float WinchSpeed = -0.50f;     		// speed of motor for winch
	float ServoBallFeed_IN = 190;		// servo position to block balls
	float ServoBallFeed_OUT = 120;		// servo position to pass balls
	//e4t = 1440 PPR
	// DistancePerPulse = (wheeldiameter * 3.1415)/360
	// = 6 * 3.1415/360 = .01308
	float DistancePerPulse = 0.05911; 	// distance per pulse for e4t encoder
	float ShooterMinDistance = 48;    	//min distance to scale shooter speed
	float ShooterMinSpeed = -0.3;		//min speed to shoot min distance
	float ShooterMaxDistance = 240;    	//max distance to scale shooter speed
	float ShooterMaxSpeed = -1.0;		//max speed to shoot max distance

	Profile AutoSteer;
	std::string LastError = "";
	std::string NowError = "";
	uint64_t AutoTime = 0;
	uint64_t TriggerTime = 0;
	uint64_t GearHangTime = 0;
	uint64_t GearReverseTime = 0;
	uint64_t GearForwardTime = 0;
	uint64_t BallFeedTime = 0;
	uint64_t BallFeedSlowTimeSP = 2000;
	uint64_t BallServoTime = 0;
	uint64_t BallFeedFastTimeSP = 500;
	uint64_t BallServoCycleTimeSP = 1000;
	bool BallFeedFast = false;
	uint64_t ElapsedTime = 0;
	float Sonar_VoltsPerInch = 0.0098;
	bool SteerReversed = false;
	float SpeedCalibration = 0.0f;
	float HeadingOffset = 0.0f;
	bool PixyOnTarget = false;
	bool PixyHasTarget = false;
	double pixyOffset = 0;
	double pixyX = 0; // 0 - 320
	double pixyY = 0; // 0 - 200
	AHRS *ahrs; //navX MXP

public:
	Robot() :
	MotorLeftSide(0),
	MotorRightSide(1),
	MotorIntake(2),
	MotorShooter(3),
	MotorWinch(5),
	ServoBallFeed(6),
	MotorGearFeed(7,1),
	DriveTrain(MotorLeftSide,MotorRightSide),
	LeftEncoder(0, 1, true, CounterBase:: k4X),   //DIO 0 and 1 channels
	RightEncoder(2, 3, true, CounterBase:: k4X),  //DIO 2 and 3 channels
	StickLeft(0),
	StickRight(1),
	PDP(0),
	Sonar(0),
	GearHomeSwitch(4),
	PixyCam(),
	Chooser(),
	Chooser2(),
	Prefs()
	{
		//instantiate the navx
		try
		{
	        ahrs = new AHRS(SPI::Port::kMXP,60);
        }
		catch (std::exception& ex )
        {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
        if ( ahrs )
        {
        	printf("NAVX Initialized OK\n");
            LiveWindow::GetInstance()->AddSensor("AHRS", "Gyro", ahrs);
        }
	}

	void RobotInit()
	{
		printf("[RobotInit]\n");
		SmartDashboard::PutString("Code Version","1.94");
		DriveTrain.SetSafetyEnabled(false);
		MotorShooter.SetSafetyEnabled(false);
		MotorIntake.SetSafetyEnabled(false);
		MotorWinch.SetSafetyEnabled(false);
		MotorGearFeed.SetSafetyEnabled(false);
		//choices for autonomous mode
		Chooser.AddDefault(AutoDoNothing, AutoDoNothing);
		Chooser.AddObject(AutoCrossBaseLine1, AutoCrossBaseLine1);
		Chooser.AddObject(AutoCrossBaseLine2, AutoCrossBaseLine2);
		Chooser.AddObject(AutoCrossBaseLine3, AutoCrossBaseLine3);
		Chooser.AddObject(AutoHangGear1, AutoHangGear1);
		Chooser.AddObject(AutoHangGear1Passive, AutoHangGear1Passive);
		Chooser.AddObject(AutoHangGear1AndShoot, AutoHangGear1AndShoot);
		Chooser.AddObject(AutoHangGear2, AutoHangGear2);
		Chooser.AddObject(AutoHangGear2Passive, AutoHangGear2Passive);
		Chooser.AddObject(AutoHangGear3, AutoHangGear3);
		Chooser.AddObject(AutoHangGear3Passive, AutoHangGear3Passive);
		Chooser.AddObject(AutoHangGear3AndShoot, AutoHangGear3AndShoot);
		Chooser.AddObject(AutoShoot1, AutoShoot1);
		Chooser.AddObject(AutoShoot3, AutoShoot3);
		SmartDashboard::PutData("Auto Modes", &Chooser);

		//choices for shooter speed control
		Chooser2.AddDefault(ShooterSpeedManual, ShooterSpeedManual);
		Chooser2.AddObject(ShooterSpeedAuto, ShooterSpeedAuto);
		Chooser2.AddObject(ShooterSpeedFixed, ShooterSpeedFixed);
		SmartDashboard::PutData("Shooter Speed Control", &Chooser2);


		Prefs = Preferences::GetInstance();
		if (!Prefs->ContainsKey("ShooterSpeed"))
			Prefs->PutDouble("ShooterSpeed",-1.0);
		if (!Prefs->ContainsKey("ShooterSpinupDelay"))
			Prefs->PutDouble("ShooterSpinupDelay",500);
		if (!Prefs->ContainsKey("ShooterMinDistance"))
			Prefs->PutDouble("ShooterMinDistance",48);
		if (!Prefs->ContainsKey("ShooterMinSpeed"))
			Prefs->PutDouble("ShooterMinSpeed",-0.3);
		if (!Prefs->ContainsKey("ShooterMaxDistance"))
			Prefs->PutDouble("ShooterMaxDistance",240);
		if (!Prefs->ContainsKey("ShooterMaxSpeed"))
			Prefs->PutDouble("ShooterMaxSpeed",-1.0);
		if (!Prefs->ContainsKey("IntakeSpeed"))
			Prefs->PutDouble("IntakeSpeed",-0.50);
		if (!Prefs->ContainsKey("WinchSpeed"))
			Prefs->PutDouble("WinchSpeed",-0.50);
		if (!Prefs->ContainsKey("BallFeedSlowTime"))
			Prefs->PutDouble("BallFeedSlowTime",2000);
		if (!Prefs->ContainsKey("BallFeedFastTime"))
			Prefs->PutDouble("BallFeedFastTime",500);
		if (!Prefs->ContainsKey("BallServoCycleTime"))
			Prefs->PutDouble("BallServoCycleTime",1000);
		LeftEncoder.SetDistancePerPulse(DistancePerPulse);
		RightEncoder.SetDistancePerPulse(DistancePerPulse);

		//INVERT MOTOR DIRECTION HERE
		MotorLeftSide.SetInverted(true);
		MotorRightSide.SetInverted(true);
		//must invert gains to follow motor direction
		if(MotorLeftSide.GetInverted())
		{
			AutoSteer.SteerKp = -0.01;
			AutoSteer.TurnKp = -0.05;
		}
		else
		{
			AutoSteer.SteerKp = 0.01;
			AutoSteer.TurnKp = 0.05;
		}
		AutoSteer.Initialize();
		MotorShooter.SetInverted(false);
		MotorIntake.SetInverted(true);
		MotorWinch.SetInverted(true);
		MotorGearFeed.SetInverted(true);
		SmartDashboard::PutNumber("Shooter Distance",0);
		SmartDashboard::PutNumber("Shooter Speed",0);
		SmartDashboard::PutNumber("pixy.X",0);
		SmartDashboard::PutNumber("pixy.Y",0);
		SmartDashboard::PutNumber("Heading",0);
		SmartDashboard::PutNumber("Distance",0);
		SmartDashboard::PutString("Drive Direction","???");
		SmartDashboard::PutString("Gear Position","???");
	}

	double GetHeading()
	{
		double offsetYaw = ahrs->GetYaw() - HeadingOffset;
		if(offsetYaw > 180) offsetYaw -= 360;
		else if(offsetYaw < -180) offsetYaw  += 360;
		if(offsetYaw < 0) offsetYaw += 360;
		return offsetYaw;
	}

	void ZeroHeading()
	{
		double offset;
		offset = ahrs->GetYaw();
		if(offset > 180) offset -= 360;
		else if(offset < -180) offset  += 360;
		if(offset < 0) offset += 360;
		HeadingOffset = offset;
	}

	void GetPixy()
	{
		int count = 0;
		static int pixyCountSinceLastBlock;

		pixyCountSinceLastBlock++;
		count = PixyCam.getBlocks(2);
		if(count > 0)
		{
			if(count > 1)
			{
				double pixyAvgX = (PixyCam.blocks[0].x + PixyCam.blocks[1].x)/2;
				pixyX = filter(pixyAvgX,pixyX,0.5);
			}
			else
				pixyX = filter(PixyCam.blocks[0].x,pixyX,0.5);

			pixyY = filter(PixyCam.blocks[0].y,pixyY,0.5);
			pixyCountSinceLastBlock = 0;
		}
		if (pixyCountSinceLastBlock < 50)  //had a target block in last second
		{
			PixyHasTarget = true;
			if(pixyX >= 150 && pixyX <= 170)
				PixyOnTarget = true;
			else
				PixyOnTarget = false;
		}
		else PixyHasTarget = false;
		SmartDashboard::PutBoolean("Pixy On Target",PixyOnTarget);
		SmartDashboard::PutNumber("pixy.X",pixyX);
		SmartDashboard::PutNumber("pixy.Y",pixyY);
	}

	void AutonomousInit() override
	{
		printf("[AutonomousInit]\n");
		//zero the navx
		ZeroHeading();
		//zero the encoders
		LeftEncoder.Reset();
		RightEncoder.Reset();
		ServoBallFeed.SetAngle(ServoBallFeed_IN);
		ShooterSpinupDelay = Prefs->GetDouble("ShooterSpinupDelay");
		if (ShooterSpinupDelay > 5000) ShooterSpinupDelay = 5000;
		if (ShooterSpinupDelay < 0) ShooterSpinupDelay = 0;
		//get autonomous selection from smartdashboard
		AutoSelected = Chooser.GetSelected();
		std::cout << "Auto selected: " << AutoSelected << std::endl;
		AutoSteer.ClearProfile();
		AutoSteer.ProfileLoaded = false;
		//DEFINE AUTONOMOUS PROFILES HERE
		//baseline is 93"
		//1 and 3 positions are 90" from center of field to center of robot
		//robot always uses 0-360 degree, with 0 being direction pointed at startup
		if(AutoSelected == AutoHangGear1 || AutoSelected == AutoHangGear1Passive || AutoSelected == AutoHangGear1AndShoot)
		{
			AutoSteer.AddStep(1,-0.5,81,0,0);    //MOVE, half speed, 78 inches
			AutoSteer.AddStep(2,-0.35,60,0,0); //TURN, quarter speed, 60 deg right
			AutoSteer.AddStep(1,-0.5,31,0,0);    //MOVE, half speed, 31 inches
			AutoSteer.AddStep(1,-0.35,8,0,0);    //MOVE, quarter speed, 12 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoHangGear2 || AutoSelected == AutoHangGear2Passive)
		{
			AutoSteer.AddStep(1,-0.5,58,0,0);    //MOVE, half speed, 58 inches
			AutoSteer.AddStep(1,-0.35,12,0,0);    //MOVE, quarter speed, 12 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoHangGear3 || AutoSelected == AutoHangGear3Passive || AutoSelected == AutoHangGear3AndShoot)
		{
			AutoSteer.AddStep(1,-0.5,77,0,0);    //MOVE, half speed, 75 inches
			AutoSteer.AddStep(2,-0.35,300,0,0);  //TURN, quarter speed, 60 deg left
			AutoSteer.AddStep(1,-0.5,31,0,0);    //MOVE, half speed, 31 inches
			AutoSteer.AddStep(1,-0.35,6,0,0);    //MOVE, quarter speed, 12 inches
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoShoot1)
		{
			AutoSteer.AddStep(1,-0.5,75,0,0);    //MOVE, half speed, past baseline
			AutoSteer.AddStep(2,-0.35,218,0,0);  //TURN, quarter speed, toward boiler
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoShoot3)
		{
			AutoSteer.AddStep(1,-0.5,75,0,0);    //MOVE, half speed, past baseline
			AutoSteer.AddStep(2,-0.35,142,0,0);  //TURN, quarter speed, toward boiler
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoCrossBaseLine1 || AutoSelected == AutoCrossBaseLine3)
		{
			AutoSteer.AddStep(1,-0.5,75,0,0);    //MOVE, half speed, past baseline
			AutoSteer.ProfileLoaded = true;
		}
		if(AutoSelected == AutoCrossBaseLine2)
		{
			AutoSteer.AddStep(1,-0.5,63,0,0);    //MOVE, half speed, past baseline
			AutoSteer.ProfileLoaded = true;
		}
		AutoTime = GetFPGATime();
	}

	uint16_t filter(uint16_t raw, uint16_t current, double lpf)
	{
		return (uint16_t)(lpf * raw) + ((1-lpf) * current);
	}

	void AutonomousPeriodic()
	{
		static bool triggerFlag = false;
		static uint8_t gearHangState = 0;
		//static bool ballFeedCycleFlag = false;
		static bool GearHangDone = false;
		double angle = GetHeading();
		double outputM, curve;
		double distance = RightEncoder.GetDistance() * -1;

		GetPixy();
		SmartDashboard::PutNumber("Heading",angle);
		SmartDashboard::PutNumber("Distance",distance);
		SmartDashboard::PutNumber("StepCount",AutoSteer.CurrentStep);
		SmartDashboard::PutBoolean("StepsDone",AutoSteer.StepsDone);
		SmartDashboard::PutNumber("GearHangState",gearHangState);
		//execute autonomous profile setup in AutonomousInit
		if(AutoSteer.ProfileLoaded)
		{
			AutoSteer.ExecuteProfile(angle,distance,pixyX,PixyHasTarget);
			//these will be set to 0 after profile is finished
			//can be set below to move if needed
			outputM = AutoSteer.OutputMagnitude;
			curve = AutoSteer.Curve;
			//if we need to hang gear after movements are done
			if((AutoSelected == AutoHangGear1 || AutoSelected == AutoHangGear2 || AutoSelected == AutoHangGear3 || AutoSelected == AutoHangGear1AndShoot || AutoSelected == AutoHangGear3AndShoot) && AutoSteer.StepsDone)
			{
				if(gearHangState == 0)
				{
					GearHangDone = false;
					gearHangState = 1;
					GearHangTime = GetFPGATime();
					printf("GearRelease Extend\n");
					//rotate gear hanger forward to release position
					MotorGearFeed.Speed = 0.5;		//half speed forward
					MotorGearFeed.TickTarget = 50;  //100 degrees
				}
				if(gearHangState == 1)  //wait for gear hanger to reach release position
				{
					ElapsedTime = GetFPGATime();
					ElapsedTime = (ElapsedTime - GearHangTime) / 1000;
					if(ElapsedTime >= 2000 )
					{
						gearHangState = 2;
						printf("Reverse Robot @ %ju\n",ElapsedTime);
					}
				}
				if(gearHangState == 2)  //back the robot away
				{
					ElapsedTime = GetFPGATime();
					ElapsedTime = (ElapsedTime - GearHangTime) / 1000;
					if(ElapsedTime < 2900 )
					{
						outputM = 0.5;
						curve = 0.0;
					}
					else
					{
						gearHangState = 3;
						printf("Stop Reversing\n");
					}

				}
				if(gearHangState == 3)  //retract gear hanger
				{
					gearHangState = 4;
					printf("GearRelease Retract\n");
					MotorGearFeed.Speed = -0.75;	//high speed reverse
					MotorGearFeed.TickTarget = 50;  //100 degrees
				}
				if(gearHangState == 4 && !GearHangDone)  //rotate if needed
				{
					if(AutoSelected == AutoHangGear1AndShoot)
					{
						if (angle < 227)
						{
							outputM = -0.6;
							curve = 0.60;
						}
						else
							GearHangDone = true;
					}
					if(AutoSelected == AutoHangGear3AndShoot)
					{
						if (angle > 147)
						{
							outputM = -0.6;
							curve = -0.60;
						}
						else
							GearHangDone = true;
					}
				}

				//run the gear feed motor when set above
				//will clear itself when position is reached
				MotorGearFeed.Execute();
			}
			//if we need to shoot after movements are done
			if(((AutoSelected == AutoShoot1 || AutoSelected == AutoShoot3) || ((AutoSelected == AutoHangGear1AndShoot || AutoSelected == AutoHangGear3AndShoot) && GearHangDone)) && AutoSteer.StepsDone)
			{
				if(!triggerFlag)
				{
					triggerFlag = true;
					TriggerTime = GetFPGATime();
					printf("Auto shoot begin\n");
				}
				//adjust this speed for where our position is
				if(AutoSelected == AutoShoot1 || AutoSelected == AutoShoot3)
					MotorShooter.Set(-0.78);
				else
					MotorShooter.Set(-0.82);
				SmartDashboard::PutNumber("Shooter Speed",MotorShooter.GetSpeed());
				//delay for shooter wheel to spinup
				ElapsedTime = GetFPGATime();
				ElapsedTime = (ElapsedTime - TriggerTime) / 1000;
				if(ElapsedTime > 1000 || (AutoSelected == AutoHangGear1AndShoot || AutoSelected == AutoHangGear3AndShoot))
				{
					MotorIntake.Set(0.75);
					ServoBallFeed.SetAngle(ServoBallFeed_OUT);
					/*
					//start servo cycle
					ElapsedTime = GetFPGATime();
					ElapsedTime = (ElapsedTime - BallFeedTime) / 1000;
					if (ElapsedTime > 200)  //adjust cycle time here
					{
						if (!ballFeedCycleFlag)
						{
							ServoBallFeed.SetAngle(ServoBallFeed_IN);
							BallServoTime = GetFPGATime();      //store start of cycle time
							ballFeedCycleFlag = true;			//flag to remember we are cycling servo
							printf("Shooter Servo Cycle Start\n");
						}
						else
						{
							ElapsedTime = GetFPGATime();
							ElapsedTime = (ElapsedTime - BallServoTime) / 1000;
							if(ElapsedTime > BallServoCycleTimeSP)
							{
								ServoBallFeed.SetAngle(ServoBallFeed_OUT);
								ballFeedCycleFlag = false;			//turn off servo cycling flag
								BallFeedTime = GetFPGATime();		//reset BallFeedTime
								printf("Shooter Servo Cycle Stop\n");
							}
						}
					}
					*/
				}
			}
			else  //not shooting so turn shooter stuff off
			{
				//spinup while turning
				if(gearHangState == 4 && !GearHangDone)
					MotorShooter.Set(-0.82);
				else
					MotorShooter.Set(0.0);
				ServoBallFeed.SetAngle(ServoBallFeed_IN);
			}
			DriveTrain.Drive(outputM,curve);
		}
		else   //not running auto profile so turn drive and shooter stuff off
		{
			DriveTrain.Drive(0.0,0.0);
			MotorShooter.Set(0.0);
			MotorIntake.Set(0.0);
			ServoBallFeed.SetAngle(ServoBallFeed_IN);
		}
		//turn these off all the time in auto
		MotorWinch.Set(0.0);
	}

	void TeleopInit()
	{
		printf("[TeleopInit]\n");
		//get the motor speeds from smartdashboard
		ShooterSpeedSelected = Chooser2.GetSelected();
		std::cout << "Shooter Control selected: " << ShooterSpeedSelected << std::endl;
		if(ShooterSpeedSelected == "ShooterSpeedFixed")
		{
			ShooterSpeed = Prefs->GetDouble("ShooterSpeed");
			if (ShooterSpeed > 0.99) ShooterSpeed = 0.99;
			if (ShooterSpeed < -0.99) ShooterSpeed = -0.99;
		}
		IntakeSpeed = Prefs->GetDouble("IntakeSpeed");
		if (IntakeSpeed > 0.99) IntakeSpeed = 0.99;
		if (IntakeSpeed <-0.99) IntakeSpeed = -0.99;
		WinchSpeed = Prefs->GetDouble("WinchSpeed");
		if (WinchSpeed > 0.99) WinchSpeed = 0.99;
		if (WinchSpeed <-0.99) WinchSpeed = -0.99;
		ServoBallFeed.SetAngle(ServoBallFeed_IN);
		//SpeedCalibration = (ShooterMaxSpeed - ShooterMinSpeed)/(ShooterMaxDistance - ShooterMinDistance);
	}

	void RunGearHanger()
	{
		static uint8_t gearHangState = 0;

		//Initiate move to Gear Receive position
		if (StickLeft.GetRawButton(11) || StickRight.GetRawButton(11))  //wait for Gear Receive button
		{
			if(gearHangState == 0)  //must not be doing gear already
			{
				MotorGearFeed.Speed = 0.5;
				MotorGearFeed.TickTarget = 14;  //30 degrees
				GearForwardTime = GetFPGATime();
				gearHangState = 10;
			}
		}

		if(gearHangState == 10)
		{
			ElapsedTime = GetFPGATime();
			ElapsedTime = (ElapsedTime - GearForwardTime) / 1000;
			if (MotorGearFeed.TickTarget == 0 || ElapsedTime > 2000) //wait for Gear Receive move to complete or timeout
			{
				if (!StickLeft.GetRawButton(11) && !StickRight.GetRawButton(11)) //wait for button release
				{
					MotorGearFeed.Speed = -0.75;
					MotorGearFeed.TickTarget = 100;  //reverse till it hits home switch
					GearReverseTime = GetFPGATime();
					gearHangState = 11;
				}
			}
		}

		if(gearHangState == 11)  //wait for home switch or timeout
		{
			ElapsedTime = GetFPGATime();
			ElapsedTime = (ElapsedTime - GearReverseTime) / 1000;
			if (GearHomeSwitch.Get() || ElapsedTime > 2000)
			{
				MotorGearFeed.Speed = 0.0;
				MotorGearFeed.TickTarget = 0;
				MotorGearFeed.Reset();
				gearHangState = 0;
			}
		}

		//Initiate move to Gear Release position
		if (StickLeft.GetRawButton(12) || StickRight.GetRawButton(12))  //wait for button
		{
			if(gearHangState == 0)  //must not be doing gear already
			{
				MotorGearFeed.Speed = 0.5;
				MotorGearFeed.TickTarget = 50;  //100 degrees
				GearForwardTime = GetFPGATime();
				gearHangState = 20;
			}
		}

		if(gearHangState == 20)
		{
			ElapsedTime = GetFPGATime();
			ElapsedTime = (ElapsedTime - GearForwardTime) / 1000;
			if (MotorGearFeed.TickTarget == 0 || ElapsedTime > 4000) //wait for Gear Release move to complete or timeout
			{
				if (!StickLeft.GetRawButton(12) && !StickRight.GetRawButton(12))  //wait for button release
				{
					MotorGearFeed.Speed = -0.75;
					MotorGearFeed.TickTarget = 100;  //reverse till it hits home switch
					GearReverseTime = GetFPGATime();
					gearHangState = 21;
				}
			}
		}

		if(gearHangState == 21)  //wait for home switch or timeout
		{
			ElapsedTime = GetFPGATime();
			ElapsedTime = (ElapsedTime - GearReverseTime) / 1000;
			if (GearHomeSwitch.Get() || ElapsedTime > 4000)
			{
				MotorGearFeed.Speed = 0.0;
				MotorGearFeed.TickTarget = 0;
				MotorGearFeed.Reset();
				gearHangState = 0;
			}
		}
		MotorGearFeed.Execute();
		SmartDashboard::PutNumber("GearHangState",gearHangState);
	}

	void TeleopPeriodic()
	{
		float leftRaw = 0.0f;         // left side joystick input
		float rightRaw = 0.0f;        // right side joystick input
		//float distanceToTarget = 0.0f;
		float throttle = 0.0f;
		float tmpWinchSpd = 0.0f;
		static bool triggerFlag = false;
		static bool ballFeedFlag = false;
		//static bool gearFeedFlag = false;
		//static bool gearFeedFlag2 = false;
		//static bool gearHomeFlag = false;

		SmartDashboard::PutNumber("Heading",GetHeading());
		//run shooter, ballfeed and agitator when right joystick trigger pulled
		if (StickRight.GetRawButton(1))
		{
			if(!triggerFlag)
			{
				triggerFlag = true;
				TriggerTime = GetFPGATime();
				printf("Shooter Trigger Pulled\n");
			}
			if (StickRight.GetRawButton(3))
			{
					ServoBallFeed.SetAngle(ServoBallFeed_OUT);
			}
			else
			{
					ServoBallFeed.SetAngle(ServoBallFeed_IN);
			}

			//distanceToTarget = filter(Sonar.GetAverageVoltage() / Sonar_VoltsPerInch,distanceToTarget,0.7);
			//SmartDashboard::PutNumber("Shooter Distance",distanceToTarget);
			if(ShooterSpeedSelected == "ShooterSpeedFixed")
			{
				MotorShooter.Set(ShooterSpeed);
			}
			//if(ShooterSpeedSelected == "ShooterSpeedAuto")
			//{
			//	//get speed = distance * calibration slope + speed offset
			//	MotorShooter.Set((distanceToTarget * SpeedCalibration) + ShooterMinSpeed);
			//}
			if(ShooterSpeedSelected == "ShooterSpeedManual" || ShooterSpeedSelected == "ShooterSpeedAuto")
			{
				//normalize throttle to 0 to -1 - forward only
				throttle = StickRight.GetRawAxis(3);
				throttle = (throttle - 1)/2;
				MotorShooter.Set(throttle);
			}
			SmartDashboard::PutNumber("Shooter Speed",MotorShooter.GetSpeed());
		}
		else
		{
			if(triggerFlag)
			{
				triggerFlag = false;
				printf("Shooter Trigger Released\n");
			}
			MotorShooter.Set(0.0);						//turn off shooter motor
			ballFeedFlag = false;
			ServoBallFeed.SetAngle(ServoBallFeed_IN);   //move plunger in to block ball feed
		}

		//run intake when left joystick trigger pulled
		if (StickLeft.GetRawButton(1))
			MotorIntake.Set(IntakeSpeed);
		else
			MotorIntake.Set(0.0);

		RunGearHanger();
		/*
		//zero the GearFeed motor when we hit home position
		if (GearHomeSwitch.Get() && !gearHomeFlag)
		{
			gearHomeFlag = true;
			MotorGearFeed.Reset();
		}
		if (!GearHomeSwitch.Get() && gearHomeFlag)
			gearHomeFlag = false;
		if (gearHomeFlag) SmartDashboard::PutString("Gear Position","HOME");
		else SmartDashboard::PutString("Gear Position","MOVING");

		//run gear holder to feed position
		if ((StickLeft.GetRawButton(11) || StickRight.GetRawButton(11)) && (!gearFeedFlag && !gearFeedFlag2) && MotorGearFeed.TickTarget == 0)
		{
			gearFeedFlag = true;
			printf("GearFeed Out\n");
			MotorGearFeed.Speed = 0.5;		//half speed forward
			MotorGearFeed.TickTarget = 14;  //30 degrees
		}
		//run gear holder to travel position
		if (!StickLeft.GetRawButton(11) && !StickRight.GetRawButton(11) && gearFeedFlag && MotorGearFeed.TickTarget == 0)
		{
			gearFeedFlag = false;
			printf("GearFeed Retract\n");
			MotorGearFeed.Speed = -0.75;	//high speed reverse
			MotorGearFeed.TickTarget = 14;  //30 degrees
		}
		//run gear holder to release position
		if ((StickLeft.GetRawButton(12) || StickRight.GetRawButton(12)) && (!gearFeedFlag && !gearFeedFlag2) && MotorGearFeed.TickTarget == 0)
		{
			gearFeedFlag2 = true;
			printf("GearRelease Out\n");
			MotorGearFeed.Speed = 0.5;		//half speed forward
			MotorGearFeed.TickTarget = 50;  //100 degrees
		}
		//run gear holder to travel position
		if (!StickLeft.GetRawButton(12) && !StickRight.GetRawButton(12) && gearFeedFlag2 && MotorGearFeed.TickTarget == 0)
		{
			gearFeedFlag2 = false;
			printf("GearRelease Retract\n");
			MotorGearFeed.Speed = -0.75;	//high speed reverse
			MotorGearFeed.TickTarget = 50;  //100 degrees
		}
		//run gear holder to travel position
		if ((StickLeft.GetRawButton(7) || StickRight.GetRawButton(7)) && MotorGearFeed.TickTarget == 0)
		{
			gearFeedFlag = false;
			printf("Gear Retract\n");
			MotorGearFeed.Speed = -0.75;	//high speed reverse
			MotorGearFeed.TickTarget = 14;  //30 degrees
		}
		MotorGearFeed.Execute();
		*/

		//default winch motor off
		tmpWinchSpd = 0.0f;
		if (StickLeft.GetRawButton(9))
			tmpWinchSpd = 0.35;
		//pull in on winch when left joystick button 3 pressed
		if (StickLeft.GetRawButton(3))
			tmpWinchSpd = WinchSpeed;
		//reverse winch when both joystick button 4 pressed
		if (StickLeft.GetRawButton(4) && StickRight.GetRawButton(4))
			tmpWinchSpd = WinchSpeed/2.0 * -1;
		MotorWinch.Set(tmpWinchSpd);

		//toggle drive direction
		SteerReversed = StickLeft.GetRawAxis(3) > 0;  //throttle position is down or +1
		if (SteerReversed) SmartDashboard::PutString("Drive Direction","REVERSED");
		else SmartDashboard::PutString("Drive Direction","NORMAL");

		//get raw joystick values for driving
		leftRaw = StickLeft.GetRawAxis(1);
		rightRaw = StickRight.GetRawAxis(1);
		//protect from out of range inputs
		if (leftRaw > 0.99) leftRaw = 0.99;
		if (leftRaw <-0.99) leftRaw = -0.99;
		if (rightRaw > 0.99) rightRaw = 0.99;
		if (rightRaw < -0.99) rightRaw = -0.99;
		if(SteerReversed)
		{
			leftRaw = leftRaw * -1;
			rightRaw = rightRaw * -1;
			DriveTrain.TankDrive(rightRaw,leftRaw);
		}
		else DriveTrain.TankDrive(leftRaw,rightRaw);

		GetPixy();
	}

	void TestInit() override
	{
		printf("[TestInit]\n");
	}

	void TestPeriodic()
	{
		LW->Run();
	}

	void DisabledInit() override
	{
		printf("[Robot was DISABLED]\n");
	}
};

START_ROBOT_CLASS(Robot)
