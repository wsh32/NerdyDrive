/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package org.usfirst.frc.team687.robot;
import org.usfirst.frc.team687.robot.util.NerdyIntegrator;

/**
 *
 * @author Wesley
 */
public class NerdyDrivePID {
    private final static double kP = 0.00444444;
    private final static double kI = 0.0000444444;
    private static double heading = 0;
    private static double error = 0;
    private static double lastError;
    private static double integration = 0;
    private static double tolerance = 2;
    
    public static void setTolerance(double t)   {
        tolerance = t;
    }
    
    public static void setHeading(double head)   {
        heading = (head+360.0)%360;
    }
    
    private static double shortestRotation(double desired)  {
        double e = heading - desired;
        if(e > 180)    {
            e = -(Math.abs(360 - e)%180);
        }   else if(e < -180)   {
            e = Math.abs(360 + e)%180;
        }
        
        return e;
    }
    
    public static double getPID(double desired) {
        lastError = error;
        error = shortestRotation(desired);
        double p = error * kP;
        integration += ((error + lastError)/2)*.02;
        double i = integration * kI;
        
        if(Math.abs(error) < tolerance)   {
            integration = 0;
            return 0;
        }   else    {
            return p + i;
        }
    }
}
