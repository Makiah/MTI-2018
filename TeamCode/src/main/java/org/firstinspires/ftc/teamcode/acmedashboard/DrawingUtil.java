package org.firstinspires.ftc.teamcode.acmedashboard;


import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.robot.structs.Pose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import hankutanku.math.vector.CartesianVector;
import hankutanku.math.vector.Vector;

/**
 * Created by ryanbrott on 1/17/18.
 */

public class DrawingUtil {
    /*
    private static List<Vector> getFrontLeftWheelContour(Pose wheelPose) {
        List<CartesianVector> wheelPath = Arrays.asList(
                new CartesianVector(-3, -2),
                new CartesianVector(3, -2),
                new CartesianVector(3, 2),
                new CartesianVector(-3, 2),
                new CartesianVector(-3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotateBy(wheelPose.heading).add(wheelPose.position));
        }
        return wheelPath;
    }

    private static List<Vector2d> getFrontLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelContour(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(3, 2),
                new Vector2d(3, -2),
                new Vector2d(-3, -2),
                new Vector2d(-3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }
    */

    public static void drawMecanumRobot(Canvas canvas, Pose robotPose) {
        canvas.setStrokeWidth(2);
        // robot body
        Vector[] bodyVectors = {
                new CartesianVector(9, 9),
                new CartesianVector(-9, 9),
                new CartesianVector(-9, -9),
                new CartesianVector(9, -9),
                new CartesianVector(9, 0),
                new CartesianVector(3, 0),
                new CartesianVector(9, 0),
                new CartesianVector(9, 9)
        };

        for (int i = 0; i < bodyVectors.length; i++) {
            bodyVectors[i] = bodyVectors[i].rotateBy(robotPose.heading).add(robotPose.position);
        }
        drawVectorPolyline(canvas, bodyVectors);

        /*
        // robot wheels
        List<Vector2d> wheelOrigins = Arrays.asList(
                new Vector2d(4.5, 5.5),
                new Vector2d(-4.5, 5.5),
                new Vector2d(-4.5, -5.5),
                new Vector2d(4.5, -5.5)
        );
        for (int i = 0; i < wheelOrigins.size(); i++) {
            Vector2d adjustedOrigin = wheelOrigins.get(i).rotated(robotPose.heading()).added(robotPose.pos());
            if (i % 2 == 0) {
                drawVectorPolyline(canvas, getFrontLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.heading())));
                drawVectorPolyline(canvas, getFrontLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.heading())));
            } else {
                drawVectorPolyline(canvas, getRearLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.heading())));
                drawVectorPolyline(canvas, getRearLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.heading())));
            }
        }*/
    }

    private static void drawVectorPolyline(Canvas canvas, Vector[] vectors) {
        double[] xCoords = new double[vectors.length];
        double[] yCoords = new double[vectors.length];
        for (int i = 0; i < xCoords.length; i++) {
            xCoords[i] = vectors[i].x();
            yCoords[i] = vectors[i].y();
        }
        canvas.strokePolyline(xCoords, yCoords);
    }

    /*
    public static void drawTrajectory(Canvas canvas, Trajectory trajectory) {
        canvas.setStrokeWidth(3);
        for (TrajectorySegment motionSegment : trajectory.segments()) {
            if (motionSegment instanceof ParametricSegment) {
                drawParametricPath(canvas, ((ParametricSegment) motionSegment).path());
            }
        }
    }

    public static void drawParametricPath(Canvas canvas, ParametricPath path) {
        if (path instanceof CompositePath) {
            for (ParametricPath subPath : ((CompositePath) path).segments()) {
                drawParametricPath(canvas, subPath);
            }
        } else if (path instanceof LinePath) {
            LinePath line = (LinePath) path;
            canvas.strokeLine(line.start().x(), line.start().y(), line.end().x(), line.end().y());
        } else if (path instanceof SplinePath) {
            SplinePath spline = (SplinePath) path;
            canvas.strokeSpline(spline.knotDistance(), spline.xOffset(), spline.yOffset(), spline.headingOffset(),
                    spline.a(), spline.b(), spline.c(), spline.d(), spline.e());
        }
    }
    */
}