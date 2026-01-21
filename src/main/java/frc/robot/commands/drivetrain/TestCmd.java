package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSys;
import edu.wpi.first.wpilibj.Timer;

public class TestCmd extends Command {
        private final TestSys testSys;
        private final Timer timer;
        private final double duration = 5.0; // seconds

    public TestCmd(TestSys testSys) {
        this.testSys = testSys;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        testSys.Start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        testSys.Stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }
}
