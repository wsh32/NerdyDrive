
package org.usfirst.frc.team687.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	Joystick leftJoy, rightJoy;
	TalonSRX ftLeft, ftRight, bkLeft, bkRight;
	IMUAdvanced imu;
	
    public void robotInit() {
    	leftJoy = new Joystick(0);
    	rightJoy = new Joystick(1);
    	ftLeft = new TalonSRX(2);
    	ftRight = new TalonSRX(3);
    	bkLeft = new TalonSRX(4);
    	bkRight = new TalonSRX(5);
    	imu = new IMUAdvanced(new SerialPort(57600,SerialPort.Port.kMXP));
    	
    	boolean is_calibrating = imu.isCalibrating();
        while(is_calibrating)   {
            Timer.delay(0.3);
            imu.zeroYaw();
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	NerdyDrive.setHeader(imu.getYaw());
        boolean beta = rightJoy.getRawButton(4);
        double fl, fr, bl, br;
        if(beta)	{
        	NerdyDrive.driveBeta(leftJoy.getX(), -leftJoy.getY(), rightJoy.getX(), -rightJoy.getY());
        }	else	{
        	NerdyDrive.driveAlpha(leftJoy.getX(), -leftJoy.getY(), rightJoy.getX());
        }
        
        fl = NerdyDrive.getFtLeft();
        fr = NerdyDrive.getFtRight();
        bl = NerdyDrive.getBkLeft();
        br = NerdyDrive.getBkRight();
        
        ftLeft.set(fl);
        ftRight.set(fr);
        bkLeft.set(bl);
        bkRight.set(br);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
