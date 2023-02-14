import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import kotlin.math.*

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(52.0, 52.0, 0.0, 250.0.toRadians, 250.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 12.0

    private const val halfWidth = 7.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private const val halfMat = 12.0

    private val startPose = Pose2d(-halfMat * 6 + 7.0, -halfMat * 3, (-90.0).toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(startPose, startPose.heading, combinedConstraints)


        val armLength = 7.0 // in inch
        val alpha = 35.0
        val beta = (90.0 - alpha) / 2.0

        // add calculate equations here
        val eg = halfMat
        val cg = eg / cos(alpha.toRadians)
        val ag = armLength
        val ac = cg - ag
        val ao = ac * tan(beta.toRadians)
        val bo = ao
        val fo = ao
        val ax = armLength * sin(alpha.toRadians)
        val gx = armLength * cos(alpha.toRadians)
        val bc = ac
        val ce = eg * tan(alpha.toRadians)
        val be = ce - bc
        val co = ac / cos(beta.toRadians)
        val cVx = -eg * 2 + ce
        val cf = co - fo
        val fh = cf * bo / co
        val ch = cf * bc / co


        // update below 6 variable values
        val Ax = ax - eg * 2
        val Ay = -eg * 2 - gx
        val Fx = cVx - ch
        val Fy = fh - eg * 3
        val Bx = be - eg * 2
        val By = -eg * 3


        val bB = Pose2d(Bx, By, (-90.0).toRadians)
        val fF = Pose2d(Fx, Fy, (-90.0/2.0 + alpha/2.0 - 45.0).toRadians)
        val aA = Pose2d(Ax +0.0, Ay+0.0, (-90 + alpha).toRadians)

        // use circle function for points - xX1
        val R = ao
        val Ox = Bx
        val Oy = By + R
        val X1 = (Bx+Fx)/2
        val Y1 = -sqrt(R*R - (X1-Ox)*(X1-Ox)) + Oy
        val EndT1 = acos((Oy - Y1) / R)
        // circle function: (X -Ox)^2 + (Y - Oy)^2 = R^2
        // or:  Y = sqrt(R^2 - (X-Ox)^2) + Oy
        // endTangent: acos((Oy - Y)/R)
        val xX1 = Pose2d(X1, Y1, (-90.0 + alpha / 4.0).toRadians)

        print(" / Ox = ")
        print( Ox)
        print(" / Oy = ")
        print( Oy)
        print(" / X1 = ")
        print( X1)
        print(" / Y1 = ")
        print( Y1)
        print("  / ")
        print(" / endt = ")
        print( Math.toDegrees(EndT1))
        print("  / ")


        // Small Example Routine
        builder1
            .strafeLeft(24.0)
            .splineToSplineHeading(bB, 0.0)
            .splineToSplineHeading(xX1, EndT1)

            .splineToSplineHeading(fF, (90.0 + alpha).toRadians / 2.0)
            .splineToSplineHeading(aA, (90.0 +  alpha).toRadians);

        list.add(builder1.build())

        // second path

        val B2x = -halfMat * 4 + halfMat / 2.0
        val B2y = -halfMat * 4 + halfMat / 2.0
        val B2angle = (180 - 45.0).toRadians
        val B2 = Pose2d(B2x, B2y, B2angle)

        val O2x = (-24 + B2x) / 2.0
        val O2y = (-24 + B2y)/ 2.0
        val R2 = (24 - 6) * sqrt(2.0) / 2

        val Gx = -24.0
        val Gy = -24.0
        val Gangle = -55.0.toRadians
        val G = Pose2d(Gx, Gy, Gangle)

        /*
        val X1 = (Bx+Fx)/2
        val Y1 = -sqrt(R*R - (X1-Ox)*(X1-Ox)) + Oy
        val EndT1 = acos((Oy - Y1) / R)
        */

        val P1x = B2x + 6
        val P1y = -sqrt(R2*R2 - (P1x-O2x)*(P1x-O2x)) + O2y
        val P1angle = -acos((O2y - P1y) / R2) + Math.PI
        val P1 = Pose2d(P1x, P1y, P1angle)

        val P2x = B2x + 12
        val P2y = -sqrt(R2*R2 - (P2x-O2x)*(P2x-O2x)) + O2y
        val P2angle = acos((O2y - P2y) / R2) + Math.PI
        val P2 = Pose2d(P2x, P2y, P2angle)


        print(" / O2x = ")
        print( O2x)
        print(" / O2y = ")
        print( O2y)
        print(" / P1x = ")
        print( P1x)
        print(" / P1y = ")
        print( P1y)
        print("  / R2 = ")
        print( R2)

        print(" / P1angle = ")
        print( Math.toDegrees(P1angle))
        print(" / P2x = ")
        print( P2x)
        print(" / P2y = ")
        print( P2y)
        print(" / P2angle = ")
        print( Math.toDegrees(P2angle))
        print("  / ")


        val startPose1 = Pose2d(-halfMat * 6 + 7.0, -halfMat * 3, (90.0).toRadians)

        val builder2 = TrajectoryBuilder(startPose1, startPose1.heading, combinedConstraints)
        builder2
            .strafeRight(12.0 - halfWidth)
            .splineToSplineHeading(B2, B2angle + Math.PI)
            //.splineToSplineHeading(P1, P1angle + Math.PI)
            //.splineToSplineHeading(P2, P2angle + Math.PI)
            //.splineToSplineHeading(G, Gangle + Math.PI)
            .splineTo(Vector2d(P1x, P1y), P1angle + Math.PI)
            .splineTo(Vector2d(P2x, P2y), P2angle + Math.PI)
            //.splineTo(Vector2d(Gx, Gy), Gangle + Math.PI)
            .splineTo(Vector2d(aA.x, aA.y), aA.heading + Math.PI);
        list.add(builder2.build())

        val builder3 = TrajectoryBuilder(startPose1, startPose1.heading, combinedConstraints)

        builder3
            .strafeRight(12.0 - halfWidth)
            .splineToSplineHeading(B2, B2angle + Math.PI)
            //.splineToSplineHeading(P1, P1angle + Math.PI)
            //.splineToSplineHeading(P2, P2angle + Math.PI)
            //.splineToSplineHeading(G, Gangle + Math.PI)
            //.splineTo(Vector2d(P1x, P1y), P1angle + Math.PI)
            //.splineTo(Vector2d(P2x, P2y), P2angle + Math.PI)
            //.splineTo(Vector2d(Gx, Gy), Gangle + Math.PI)
            .splineTo(Vector2d(aA.x, aA.y), aA.heading + Math.PI);

        list.add(builder3.build())

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(-72.0 + 7.0, -36.0), 14.0, 15.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
