package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Util {
    
    public static void sendLambda(String name, Runnable runnable) {
        SmartDashboard.putData(lambdaAsCommand(runnable));
    }

    public static Command lambdaAsCommand(Runnable runnable) {
        return Commands.run(runnable).asProxy();
    }
}
