# How to Programming


#### Use Motors

- To import `import com.ctre.phoenix.motorcontrol.can.*;`
- To initialize `WPI_TalonSRX Motor = new WPI_TalonSRX(6);`

- To use `Motor.set(value)` with value between -1 and 1
#### Use joysticks
- To import `import edu.wpi.first.wpilibj.Joystick;`
- To initialize `Joystick Joy = new Joystick(0);`

- To get axis values `Joy.getRawAxis(0);` with value between -1 and 1
- To get button values `Joy.getRawButton(0);` with boolean output
- To get angle in radians `Math.atan2(xAxis, -yAxis)`
- To get magnitude `Math.sqrt(xAxis*xAxis + yAxis*yAxis);`

#### Use Encoders
- To import `import edu.wpi.first.wpilibj.Encoder`
- To initialize `Encoder sampleEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);`
- To set max period (secs) without clicks before it is considered at rest `sampleEncoder.setMaxPeriod(.1);`
- Minimum rate (rotation/time) before device is stopped `sampleEncoder.setMinRate(10);`
- How much distance is traveled every encoder pulse `sampleEncoder.setDistancePerPulse(5);`
- Reverse the direction encoder counts `sampleEncoder.setReverseDirection(true);`
- How many samples to take (1-127) when determining the period`sampleEncoder.setSamplesToAverage(7);`
#### Use accelerometer
- 
## Examples

#### Make Motor Spin with Joystick Input　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	WPI_TalonSRX Motor = new WPI_TalonSRX(6); //initialize motor
	Joystick Joy = new Joystick(0); // initiallize joystick (to find number, check driver station)

	public void robotInit() {
		//This function is run when the robot is first started up
	}

	public void teleopPeriodic() {
		//This function is called periodically during operator control
	    double joystickValue = Joy.getRawAxis(1); //find raw axis in driver station
		System.out.println("Joy:" + joystickValue);
        Motor.set(joystickValue*0.7); //scale down by 0.7
	}
}
```


#### Arcade Drive　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(6);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(7);
	WPI_TalonSRX _leftSlave2 = new WPI_TalonSRX(4);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);

	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick _joy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		/*
		 * take our extra talons and just have them follow the Talons updated in
		 * arcadeDrive
		 */
		_leftSlave1.follow(_frontLeftMotor);
		_leftSlave2.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_rightSlave2.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_leftSlave2.setInverted(false);
		
		_frontRightMotor.setInverted(false);
		_rightSlave1.setInverted(false);
		_rightSlave2.setInverted(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		double forward = -1.0 * _joy.getY();
		/* sign this so right is positive. */
		double turn = +1.0 * _joy.getZ();
		/* deadband */
		if (Math.abs(forward) < 0.10) {
			/* within 10% joystick, make it zero */
			forward = 0;
		}
		if (Math.abs(turn) < 0.10) {
			/* within 10% joystick, make it zero */
			turn = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("JoyY:" + forward + "  turn:" + turn );
		/* drive the robot, when driving forward one side will be red.  
		 * This is because DifferentialDrive assumes 
		 * one side must be negative */
		_drive.tankDrive(forward, turn);
	}
}
```

#### Tank Drive　

```java
package org.usfirst.frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot {

	/* talons for arcade drive */
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(6);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(2);

	/* extra talons and victors for six motor drives */
	WPI_TalonSRX _leftSlave1 = new WPI_TalonSRX(5);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(7);
	WPI_TalonSRX _leftSlave2 = new WPI_TalonSRX(4);
	WPI_VictorSPX _rightSlave2 = new WPI_VictorSPX(17);

	DifferentialDrive _drive = new DifferentialDrive(_frontLeftMotor, _frontRightMotor);

	Joystick leftJoy = new Joystick(0);
	Joystick rightJoy = new Joystick(1);

	//This function is run when the robot is first started up and should be used for any initialization code.
	
	public void robotInit() {
		//take our extra talons and just have them follow the Talons updated in
		_leftSlave1.follow(_frontLeftMotor);
		_leftSlave2.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_rightSlave2.follow(_frontRightMotor);

		/* drive robot forward and make sure all 
		 * motors spin the correct way.
		 * Toggle booleans accordingly.... */
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_leftSlave2.setInverted(false);
		
		_frontRightMotor.setInverted(false);
		_rightSlave1.setInverted(false);
		_rightSlave2.setInverted(false);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	
		double right = rightJoy.getY();
		double left =  leftjoy.getY();
		/* deadband */
		if (Math.abs(left) < 0.10) {
			/* within 10% joystick, make it zero */
			left = 0;
		}
		if (Math.abs(right) < 0.10) {
			/* within 10% joystick, make it zero */
			right = 0;
		}
		/* print the joystick values to sign them, comment
		 * out this line after checking the joystick directions. */
		System.out.println("Left: " + left + "  Right:" + right );
		_drive.tankDrive(left, right);
	}
}
```

