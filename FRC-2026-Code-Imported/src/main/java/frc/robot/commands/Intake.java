package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grabber;

public class Intake extends Command {
        private Grabber intakesub;
        
        public Intake(Grabber intakesub){
            this.intakesub = intakesub;
            addRequirements(intakesub);
        }
        @Override
        public void execute(){
            intakesub.Spin(1);
        }
        @Override
        public void end(boolean interrupted){
            intakesub.Spin(0);
        }
}
