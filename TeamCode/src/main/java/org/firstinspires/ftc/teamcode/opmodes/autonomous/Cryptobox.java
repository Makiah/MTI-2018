package org.firstinspires.ftc.teamcode.opmodes.autonomous;

public class Cryptobox
{
    //left-right
    private final boolean[][] glyphs;

    public enum Column {
        LEFT, CENTER, RIGHT
    }

    public Cryptobox()
    {
        this.glyphs = new boolean[3][4];
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
