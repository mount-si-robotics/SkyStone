package org.firstinspires.ftc.opmodes.competition.auto.TimeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotlib.auto.SiBorgsMecanumTimeAuto;
import org.firstinspires.ftc.robotlib.state.AutoDirection;
import org.firstinspires.ftc.robotlib.state.Button;

@Autonomous(name="Any Side | Bridge Park", group="CompAuto")
public class BasicAutoAnySidePark extends SiBorgsMecanumTimeAuto
{
    // Pathing buttons
    private Button moveForward;
    private Button moveRearward;
    private Button moveLeft;
    private Button moveRight;

    // AutoProgram
    private AutoProgram autoPath;

    @Override
    public void runOpMode()
    {
        moveForward = new Button();
        moveRearward = new Button();
        moveRight = new Button();
        moveLeft = new Button();

        init("Bridge Park");
        while (!isStarted())
        {
            updateProgramPlan();
            initLoop();
            displayProgramPlan();
        }

        switch(autoPath)
        {
            case FRONTRIGHT:
            {
                super.position(AutoDirection.FRONT, 1);
                super.position(AutoDirection.RIGHT, 1);
                break;
            }

            case FRONTLEFT:
            {
                super.position(AutoDirection.FRONT, 1);
                super.position(AutoDirection.LEFT, 1);
                break;
            }

            case LEFT:
            {
                super.position(AutoDirection.LEFT, 1);
                break;
            }

            case RIGHT:
            {
                super.position(AutoDirection.RIGHT, 1);
                break;
            }
        }
    }

    private void displayProgramPlan()
    {
        switch (autoPath)
        {
            case LEFT:
            {
                telemetry.addData("___", "");
                telemetry.addData("<-_", "");
                telemetry.addData("___", "");
                break;
            }
            case RIGHT:
            {
                telemetry.addData("___", "");
                telemetry.addData("_->", "");
                telemetry.addData("___", "");
                break;
            }
            case FRONTLEFT:
            {
                telemetry.addData("<-_", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
            case FRONTRIGHT:
            {
                telemetry.addData("_->", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
            case FORWARD:
            {
                telemetry.addData("_^_", "");
                telemetry.addData("_|_", "");
                telemetry.addData("___", "");
                break;
            }
        }
    }

    private void updateProgramPlan()
    {
        switch (autoPath)
        {
            case LEFT:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.RIGHT;
                }
                else if (moveForward.onPress())
                {
                    autoPath = AutoProgram.FRONTLEFT;
                }
                break;
            }
            case RIGHT:
            {
                if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.LEFT;
                }
                else if (moveForward.onPress())
                {
                    autoPath = AutoProgram.FRONTRIGHT;
                }
                break;
            }
            case FRONTRIGHT:
            {
                if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.FORWARD;
                }
                else if (moveRearward.onPress())
                {
                    autoPath = AutoProgram.RIGHT;
                }
                break;
            }
            case FRONTLEFT:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.FORWARD;
                }
                else if (moveRearward.onPress())
                {
                    autoPath = AutoProgram.LEFT;
                }
                break;
            }
            case FORWARD:
            {
                if (moveRight.onPress())
                {
                    autoPath = AutoProgram.FRONTRIGHT;
                }
                else if (moveLeft.onPress())
                {
                    autoPath = AutoProgram.FRONTLEFT;
                }
                break;
            }
        }
    }
}

enum AutoProgram
{
    FRONTLEFT(),
    FRONTRIGHT(),
    FORWARD(),
    LEFT(),
    RIGHT()
}
