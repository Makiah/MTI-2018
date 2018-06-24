package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import hankutanku.math.angle.Angle;
import hankutanku.math.angle.DegreeAngle;
import hankutanku.math.vector.CartesianVector;

public class Cryptobox
{
    //left-right
    private final boolean[][] glyphs;

    public enum Column {
        LEFT, CENTER, RIGHT
    }

    public final Pose[] depositPoses;
    public final Angle unmodifiedDepositAngle;

    public Cryptobox(CompetitionProgram.Alliance alliance, CompetitionProgram.BalancePlate balancePlate)
    {
        this.glyphs = new boolean[3][4];

        if (alliance == CompetitionProgram.Alliance.BLUE && balancePlate == CompetitionProgram.BalancePlate.BOTTOM)
        {
            this.depositPoses = new Pose[]{
                    new Pose(new CartesianVector(24, 52), new DegreeAngle(270 + AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(24, 60), new DegreeAngle(270 - AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(24, 68), new DegreeAngle(270 - AutonomousSettings.depositAngleOffset)),
            };

            unmodifiedDepositAngle = new DegreeAngle(270);
        }
        else if (alliance == CompetitionProgram.Alliance.BLUE && balancePlate == CompetitionProgram.BalancePlate.TOP)
        {
            this.depositPoses = new Pose[]{
                    new Pose(new CartesianVector(28, 120), new DegreeAngle(180 - AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(36, 120), new DegreeAngle(180 - AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(44, 120), new DegreeAngle(180 - AutonomousSettings.depositAngleOffset)),
            };

            unmodifiedDepositAngle = new DegreeAngle(180);
        }
        else if (alliance == CompetitionProgram.Alliance.RED && balancePlate == CompetitionProgram.BalancePlate.BOTTOM)
        {
            this.depositPoses = new Pose[]{
                    new Pose(new CartesianVector(120, 52), new DegreeAngle(270 + AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(120, 60), new DegreeAngle(270 - AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(120, 68), new DegreeAngle(270 - AutonomousSettings.depositAngleOffset)),
            };

            unmodifiedDepositAngle = new DegreeAngle(90);
        }
        else if (alliance == CompetitionProgram.Alliance.RED && balancePlate == CompetitionProgram.BalancePlate.TOP)
        {
            this.depositPoses = new Pose[]{
                    new Pose(new CartesianVector(100, 120), new DegreeAngle(180 + AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(108, 120), new DegreeAngle(180 + AutonomousSettings.depositAngleOffset)),
                    new Pose(new CartesianVector(116, 120), new DegreeAngle(180 + AutonomousSettings.depositAngleOffset)),
            };

            unmodifiedDepositAngle = new DegreeAngle(180);
        }
        else
        {
            depositPoses = null; // satisfy android studio
            unmodifiedDepositAngle = null;
        }
    }

    public Pose getDepositPoseFor(Column column)
    {
        return depositPoses[indexFromCol(column)];
    }

    private int indexFromCol(Column column)
    {
        switch (column)
        {
            case LEFT:
                return 0;
            case CENTER:
                return 1;
            case RIGHT:
                return 2;
        }

        return 0;
    }

    public void dump(Column col, int glyphs)
    {
        int startDumpIndex = 0;
        while (this.glyphs[(indexFromCol(col))][startDumpIndex])
            startDumpIndex++;

        for (int i = 0; i < glyphs; i++)
        {
            if (i >= this.glyphs[0].length)
                return;

            this.glyphs[indexFromCol(col)][i + startDumpIndex] = true;
        }
    }

    public Column getOptimalDumpLocation()
    {
        for (int i = 0; i < this.glyphs.length; i++)
        {
            for (int j = 0; j < this.glyphs[0].length; j++)
            {
                if (!this.glyphs[i][j])
                {
                    switch (i)
                    {
                        case 0:
                            return Column.LEFT;
                        case 1:
                            return Column.CENTER;
                        case 2:
                            return Column.RIGHT;
                    }
                }
            }
        }

        return null;
    }
}
