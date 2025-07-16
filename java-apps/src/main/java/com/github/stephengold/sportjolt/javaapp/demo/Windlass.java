/*
 Copyright (c) 2022-2025 Stephen Gold and Yanis Boudiaf

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.github.stephengold.sportjolt.javaapp.demo;

import com.github.stephengold.joltjni.Body;
import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.CapsuleShape;
import com.github.stephengold.joltjni.Constraint;
import com.github.stephengold.joltjni.CylinderShapeSettings;
import com.github.stephengold.joltjni.FixedConstraintSettings;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.PointConstraintSettings;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RMat44;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.RotatedTranslatedShape;
import com.github.stephengold.joltjni.RotatedTranslatedShapeSettings;
import com.github.stephengold.joltjni.Shape;
import com.github.stephengold.joltjni.StaticCompoundShapeSettings;
import com.github.stephengold.joltjni.TaperedCapsuleShapeSettings;
import com.github.stephengold.joltjni.TwoBodyConstraint;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.operator.Op;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.ConstShapeSettings;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RMat44Arg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.joltjni.std.Std;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.input.CameraInputProcessor;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.PhysicsTickListener;
import org.lwjgl.glfw.GLFW;

/**
 * A physics demo that simulates a cable coiled around a horizontal barrel. The
 * cable is composed of capsule-shaped rigid segments. A hook is attached to the
 * free end of the cable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Windlass extends BasePhysicsApp implements PhysicsTickListener {
    // *************************************************************************
    // constants

    /**
     * simulation speed when "paused"
     */
    final private static float PAUSED_SPEED = 1e-9f;
    /**
     * simulation time step (in seconds)
     */
    final private static float timeStep = 0.0015f;
    /**
     * initial number of times the cable coils around the barrel
     */
    final private static int numCoils = 4;
    /**
     * number of cable segments in the pendant section
     */
    final private static int numPendantSegments = 4;
    /**
     * number of cable segments per coil
     */
    final private static int numSegmentsPerCoil = 12;
    /**
     * number of cable segments in the initially coiled portion
     */
    final private static int numCoiledSegments = numCoils * numSegmentsPerCoil;
    // *************************************************************************
    // fields

    /**
     * body that represents the barrel
     */
    private static Body barrel;
    /**
     * All cable segments have exactly the same shape.
     */
    private static ConstShape segmentShape;
    /**
     * rotation of the barrel (in radians)
     */
    private static float barrelXRotation;
    /**
     * simulation speed (simulated seconds per wall-clock second)
     */
    private static float physicsSpeed = 1f;
    /**
     * input signal: 1&rarr;turn counter-clockwise (initially lowers the hook)
     */
    private static int signalCcw;
    /**
     * input signal: 1&rarr;turn clockwise (initially raises the hook)
     */
    private static int signalCw;
    /**
     * orientation of the barrel
     */
    final private static Quat barrelOrientation = new Quat();
    /**
     * location of the forward pivot in a segment's local coordinates
     */
    final private static Vec3 localPivot = new Vec3();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Windlass application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public Windlass() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the Windlass application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Windlass application = new Windlass();
        application.start();
    }
    // *************************************************************************
    // BasePhysicsApp methods

    /**
     * Create the PhysicsSystem. Invoked once during initialization.
     *
     * @return a new object
     */
    @Override
    protected PhysicsSystem createSystem() {
        int numBpLayers = 1; // use a single broadphase layer for simplicity
        int maxBodies = numCoiledSegments + numPendantSegments + 3;
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        // To enable the callbacks, register the application as a tick listener.
        addTickListener(this);

        result.setGravity(0f, -981f, 0f); // 100x scale

        // Reduce the timestep for better accuracy:
        this.timePerStep = timeStep;

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        super.initialize();

        setVsync(false);
        configureCamera();
        configureInput();
        setBackgroundColor(Constants.SKY_BLUE);
    }

    /**
     * Populate the {@code PhysicsSystem} with bodies and constraints. Invoked
     * once during initialization.
     */
    @Override
    protected void populateSystem() {
        float cableRadius = 1f; // should be much larger than convex radius
        RVec3 attachPoint = addBarrel(cableRadius);
        /*
         * Determine the segment length, which is also the distance between
         * successive pivots.
         */
        float deltaPhi = 2f * Jolt.JPH_PI / numSegmentsPerCoil;
        float z0 = attachPoint.z();
        float deltaX = 2.1f * cableRadius / numSegmentsPerCoil;
        float deltaY = 2f * z0 * Jolt.tan(deltaPhi / 2f);
        float segmentLength = Std.hypot(deltaX, deltaY);

        // The segment shape is a Z-axis capsule.
        assert segmentLength > 2f * cableRadius : "alternate segments collide!";
        Shape yCapsule = new CapsuleShape(segmentLength / 2f, cableRadius);
        QuatArg y2z = Quat.sEulerAngles(Jolt.JPH_PI / 2f, 0f, 0f);
        segmentShape = new RotatedTranslatedShape(new Vec3(), y2z, yCapsule);
        localPivot.set(0f, 0f, segmentLength / 2f);
        /*
         * Make the first cable segment tangent to the +Z side of the barrel
         * and attach it with a fixed constraint (all DOFs locked).
         */
        float zRotation = Jolt.aTan2(deltaX, deltaY);
        Quat orientation = Quat.sEulerAngles(0f, zRotation, 0f);
        orientation = Op.star(y2z, orientation);

        Body segment = addCableSegment(attachPoint, orientation);
        FixedConstraintSettings fixed = new FixedConstraintSettings();
        fixed.setAutoDetectPoint(true);
        TwoBodyConstraint fc = fixed.create(barrel, segment);
        physicsSystem.addConstraint(fc);

        QuatArg rotatePhi = Quat.sEulerAngles(deltaPhi, 0f, 0f);

        // Attach successive segments in a spiral coiling around the barrel:
        float phi = Jolt.JPH_PI / 2f;
        Body endSegment = segment;
        RVec3 center = new RVec3(attachPoint);
        for (int segmentI = 0; segmentI < numCoiledSegments; ++segmentI) {
            // Calculate the position of the next segment:
            center.addInPlace(deltaX, 0f, 0f);
            phi += deltaPhi;
            center.setY(z0 * Jolt.cos(phi));
            center.setZ(z0 * Jolt.sin(phi));
            orientation.set(Op.star(rotatePhi, orientation));

            // Create a new segment and splice it to the existing cable:
            Body newSegment = addCableSegment(center, orientation);
            spliceCableSegments(newSegment, endSegment);

            endSegment = newSegment;
        }

        orientation = Quat.sEulerAngles(Jolt.JPH_PI / 2f, 0f, 0f);

        // Attach successive segments in a vertical drop chain:
        for (int segmentI = 0; segmentI < numPendantSegments; ++segmentI) {
            // Calculate the location of the next segment:
            center.addInPlace(0f, -segmentLength, 0f);

            // Create a new segment and splice it to the existing cable:
            Body newSegment = addCableSegment(center, orientation);
            spliceCableSegments(newSegment, endSegment);

            endSegment = newSegment;
        }

        addHook(endSegment, cableRadius);
    }

    /**
     * Advance the physics simulation by the specified amount. Invoked during
     * each update.
     *
     * @param wallClockSeconds the elapsed wall-clock time since the previous
     * invocation of {@code updatePhysics} (in seconds, &ge;0)
     */
    @Override
    protected void updatePhysics(float wallClockSeconds) {
        float simulateSeconds = physicsSpeed * wallClockSeconds;
        super.updatePhysics(simulateSeconds);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback invoked (by Sport Jolt, not by Jolt Physics) after the system
     * has been stepped.
     *
     * @param system the system that was just stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void physicsTick(PhysicsSystem system, float timeStep) {
        // do nothing
    }

    /**
     * Callback invoked (by Sport Jolt, not by Jolt Physics) before the system
     * is stepped.
     *
     * @param system the system that's about to be stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSystem system, float timeStep) {
        // Turn the barrel based on user-input signals:
        float turnRate = 4f; // radians per second
        barrelXRotation += (signalCcw - signalCw) * turnRate * timeStep;
        barrelOrientation.set(Quat.sEulerAngles(barrelXRotation, 0f, 0f));
        RVec3Arg loc = barrel.getPosition();
        barrel.moveKinematic(loc, barrelOrientation, timeStep);
    }
    // *************************************************************************
    // private methods

    /**
     * Add the barrel, which is a kinematic rigid body shaped like a horizontal
     * cylinder, with flanges and handles at both ends.
     *
     * @param cableRadius the radius of the cable (in physics-space units,
     * &gt;0)
     * @return the attachment point for the cable (a location vector in system
     * coordinates)
     */
    private RVec3 addBarrel(float cableRadius) {
        QuatArg y2x = Quat.sEulerAngles(0f, 0f, -Jolt.JPH_PI / 2f);

        float drumLength = 12f * cableRadius;
        float drumRadius = 0.6f * drumLength;
        ConstShapeSettings cylinderShape
                = new CylinderShapeSettings(drumLength / 2f, drumRadius);
        cylinderShape = new RotatedTranslatedShapeSettings(
                new Vec3(), y2x, cylinderShape);

        float flangeRadius = drumRadius + 3.5f * cableRadius;
        float flangeWidth = 0.1f * drumLength;
        ConstShapeSettings flangeShape
                = new CylinderShapeSettings(flangeWidth / 2f, flangeRadius);
        flangeShape = new RotatedTranslatedShapeSettings(
                new Vec3(), y2x, flangeShape);

        float handleRadius = 0.8f * cableRadius;
        float handleLength = 8f * cableRadius;
        ConstShapeSettings handleShape
                = new CylinderShapeSettings(handleLength / 2f, handleRadius);
        handleShape = new RotatedTranslatedShapeSettings(
                new Vec3(), y2x, handleShape);

        StaticCompoundShapeSettings barrelShape
                = new StaticCompoundShapeSettings();
        barrelShape.addShape(new Vec3(), new Quat(), cylinderShape);

        float flangeX = (drumLength + flangeWidth) / 2f;
        barrelShape.addShape(+flangeX, 0f, 0f, flangeShape);
        barrelShape.addShape(-flangeX, 0f, 0f, flangeShape);

        float handleX = drumLength / 2f + flangeWidth + handleLength / 2f;
        float handleY = flangeRadius - handleRadius;
        barrelShape.addShape(+handleX, +handleY, 0f, handleShape);
        barrelShape.addShape(-handleX, -handleY, 0f, handleShape);

        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Kinematic)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setShapeSettings(barrelShape);
        bcs.getMassPropertiesOverride().setMass(100f);
        BodyInterface bi = physicsSystem.getBodyInterface();
        barrel = bi.createBody(bcs);
        bi.addBody(barrel, EActivation.Activate);
        visualizeShape(barrel);

        // Calculate an attachment point on the +Z side of the drum;
        double x0 = -0.49 * drumLength + cableRadius;
        double z0 = drumRadius + cableRadius;
        RVec3 result = new RVec3(x0, 0., z0);

        return result;
    }

    /**
     * Add a single segment of cable.
     *
     * @param center the desired center location (in system coordinates, not
     * null, unaffected)
     * @param orientation the desired orientation (in system coordinates, not
     * null, unaffected)
     * @return a new instance
     */
    private Body addCableSegment(RVec3Arg center, QuatArg orientation) {
        BodyCreationSettings bcs = new BodyCreationSettings()
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setPosition(center)
                .setRotation(orientation)
                .setShape(segmentShape);
        bcs.getMassPropertiesOverride().setMass(0.2f);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body result = bi.createBody(bcs);
        bi.addBody(result, EActivation.Activate);
        visualizeShape(result);

        return result;
    }

    /**
     * Attach a hook to the end of the cable.
     *
     * @param endSegment the final segment of the cable (not null)
     * @param cableRadius the radius of the cable (&gt;0)
     */
    private void addHook(Body endSegment, float cableRadius) {
        /*
         * Collision shape is composed of 11 overlapping 2-sphere shapes,
         * arranged in a circular arc.
         */
        int numChildren = 11;
        int numSpheres = numChildren + 1;
        float hookRadius = 4f * cableRadius;
        float maxThick = 2.1f * cableRadius; // max thickness
        float minThick = 0.5f * cableRadius; // min thickness

        float[] radius = new float[numSpheres];
        float[] y = new float[numSpheres];
        float[] z = new float[numSpheres];
        float xAngle = 0f; // in radians
        for (int sphereI = 0; sphereI < numSpheres; ++sphereI) {
            float p = sphereI / (float) (numSpheres - 1); // goes from 0 to 1
            float p3 = p * p * p;
            float thickness = maxThick - p3 * (maxThick - minThick);
            radius[sphereI] = thickness / 2f;
            if (sphereI > 0) {
                xAngle += radius[sphereI] / hookRadius;
            }
            y[sphereI] = hookRadius * Jolt.cos(xAngle);
            z[sphereI] = -hookRadius * Jolt.sin(xAngle);
            xAngle += radius[sphereI] / hookRadius;
        }

        StaticCompoundShapeSettings shape = new StaticCompoundShapeSettings();
        for (int childI = 0; childI < numChildren; ++childI) {
            int nextI = childI + 1;
            double dy = y[nextI] - y[childI];
            double dz = z[nextI] - z[childI];
            float halfHeight = (float) (0.5 * Math.hypot(dy, dz));

            float bottomRadius = radius[childI];
            float topRadius = radius[nextI];
            TaperedCapsuleShapeSettings twoSphere
                    = new TaperedCapsuleShapeSettings(
                            halfHeight, topRadius, bottomRadius);

            float yOffset = 0.5f * (y[childI] + y[nextI]);
            float zOffset = 0.5f * (z[childI] + z[nextI]);
            Vec3Arg offset = new Vec3(0f, yOffset, zOffset);
            QuatArg rotation
                    = Quat.sFromTo(Vec3.sAxisY(), new Vec3(0f, dy, dz));
            shape.addShape(offset, rotation, twoSphere);
        }

        // Locate the final pivot:
        RMat44 endTransform = endSegment.getWorldTransform();
        RVec3Arg pivotLocation = endTransform.multiply3x4(localPivot);

        float pivotY = hookRadius + maxThick / 2f;
        RVec3Arg center = Op.minus(pivotLocation, new Vec3(0f, pivotY, 0f));

        BodyCreationSettings bcs = new BodyCreationSettings()
                .setAngularDamping(0.7f)
                .setLinearDamping(0.4f)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setPosition(center)
                .setShapeSettings(shape);
        bcs.getMassPropertiesOverride().setMass(3f);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body hook = bi.createBody(bcs);
        bi.addBody(hook, EActivation.Activate);
        visualizeShape(hook);

        PointConstraintSettings constraint = new PointConstraintSettings();
        constraint.setPoint1(pivotLocation);
        constraint.setPoint2(pivotLocation);
        TwoBodyConstraint tbc = constraint.create(hook, endSegment);
        physicsSystem.addConstraint(tbc);
    }

    /**
     * Configure the camera and CIP during initialization.
     */
    private static void configureCamera() {
        CameraInputProcessor cip = getCameraInputProcessor();
        cip.setMoveSpeed(20f);
        cip.setRotationMode(RotateMode.DragLMB);

        cam.setAzimuth(-1.78f)
                .setLocation(30f, 25f, 135f)
                .setUpAngle(-0.28f);
    }

    /**
     * Configure keyboard input during initialization.
     */
    private void configureInput() {
        getInputManager().add(new InputProcessor() {
            @Override
            public void onKeyboard(int keyId, boolean isPressed) {
                switch (keyId) {
                    case GLFW.GLFW_KEY_DOWN:
                        signalCcw = isPressed ? 1 : 0;
                        return;

                    case GLFW.GLFW_KEY_UP:
                        signalCw = isPressed ? 1 : 0;
                        return;

                    case GLFW.GLFW_KEY_PAUSE:
                    case GLFW.GLFW_KEY_PERIOD:
                        if (isPressed) {
                            togglePause();
                        }
                        return;

                    default:
                }
                super.onKeyboard(keyId, isPressed);
            }
        });
    }

    /**
     * Connect the specified cable segments with a point constraint.
     *
     * @param newSegment the new cable segment (not null)
     * @param endSegment the final segment of the cable so far (not null)
     */
    private void spliceCableSegments(Body newSegment, Body endSegment) {
        // Position the pivot:
        RMat44Arg endTransform = endSegment.getWorldTransform();
        RVec3Arg pivotLocation = endTransform.multiply3x4(localPivot);

        PointConstraintSettings constraint = new PointConstraintSettings();
        constraint.setPoint1(pivotLocation);
        constraint.setPoint2(pivotLocation);

        Constraint tbc = constraint.create(newSegment, endSegment);
        physicsSystem.addConstraint(tbc);
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private static void togglePause() {
        physicsSpeed = (physicsSpeed <= PAUSED_SPEED) ? 1f : PAUSED_SPEED;
    }
}
