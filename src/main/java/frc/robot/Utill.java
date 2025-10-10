package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Utill {
    public static double getDashNumber(String name, double valueWhenNoCom) {
        double smartDashNum = SmartDashboard.getNumber(name, valueWhenNoCom);
        SmartDashboard.putNumber(name, smartDashNum);
        return smartDashNum;
    }
}
