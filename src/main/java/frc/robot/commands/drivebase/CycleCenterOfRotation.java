package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Drivebase.CenterOfRotation;

public class CycleCenterOfRotation extends InstantCommand {
    
    public enum Direction { UP, DOWN; }

    private Drivebase drivebase;
    private Direction direction;
    
    public CycleCenterOfRotation(Drivebase drivebase, Direction direction)
    {
        this.drivebase = drivebase;
        this.direction = direction;
    }

    @Override
    public void initialize() {

        CenterOfRotation currentCenterOfRotation = drivebase.getCenterOfRotation();
        int lengthOfEnum = CenterOfRotation.values().length;
        int index;
        if(!drivebase.isMecanum()) //immediate cancel if this is not a meccanum
        {
            drivebase.setCenterOfRotation(CenterOfRotation.CENTER);
            return;
        }
        if(direction == Direction.UP)
        {
            index = (currentCenterOfRotation.ordinal() + 1) % lengthOfEnum;
        }
        else if(direction == Direction.DOWN)
        {
            index = (currentCenterOfRotation.ordinal() + lengthOfEnum - 1) % lengthOfEnum;
        }
        else
        {
            index = currentCenterOfRotation.ordinal();
        }

        drivebase.setCenterOfRotation(CenterOfRotation.values()[index]);
    }
}
