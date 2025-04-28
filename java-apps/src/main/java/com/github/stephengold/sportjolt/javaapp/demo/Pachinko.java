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
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.BoxShapeSettings;
import com.github.stephengold.joltjni.MapObj2Bp;
import com.github.stephengold.joltjni.ObjVsBpFilter;
import com.github.stephengold.joltjni.ObjVsObjFilter;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.StaticCompoundShapeSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EAllowedDofs;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.input.CameraInputProcessor;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.PhysicsTickListener;
import org.joml.Random;
import org.lwjgl.glfw.GLFW;

/**
 * A 2-D physics demo that simulates a simple Pachinko machine.
 * <p>
 * https://en.wikipedia.org/wiki/Pachinko
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Pachinko
        extends BasePhysicsApp
        implements PhysicsTickListener {
    // *************************************************************************
    // constants

    /**
     * time interval between balls (in simulated seconds)
     */
    final private static float addInterval = 3f;
    /**
     * radius of each ball (in meters)
     */
    final private static float ballRadius = 1f;
    // *************************************************************************
    // fields

    /**
     * settings for creating balls
     */
    private static BodyCreationSettings ballBcs;
    /**
     * simulation speed (simulated seconds per wall-clock second)
     */
    private static float physicsSpeed = 1f;
    /**
     * elapsed time since a ball was added (in simulated seconds)
     */
    private static float timeSinceAdded;
    /**
     * randomize ball motion
     */
    final private static Random generator = new Random();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Pachinko application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public Pachinko() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the Pachinko application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Pachinko application = new Pachinko();
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
    public PhysicsSystem createSystem() {
        // Use 2 broadphase layers for efficiency:
        int bpLayerNonMoving = 0;
        int bpLayerMoving = 1;
        int numBpLayers = 2;
        MapObj2Bp mapObj2Bp = new MapObj2Bp(numObjLayers, numBpLayers)
                .add(objLayerNonMoving, bpLayerNonMoving)
                .add(objLayerMoving, bpLayerMoving);
        ObjVsBpFilter objVsBpFilter
                = new ObjVsBpFilter(numObjLayers, numBpLayers)
                        .disablePair(objLayerNonMoving, bpLayerNonMoving);
        ObjVsObjFilter objVsObjFilter = new ObjVsObjFilter(numObjLayers)
                .disablePair(objLayerNonMoving, objLayerNonMoving);

        int maxBodies = 1_800;
        int numBodyMutexes = 0; // 0 means "use the default value"
        int maxBodyPairs = 65_536;
        int maxContacts = 20_480;
        PhysicsSystem result = new PhysicsSystem();
        result.init(maxBodies, numBodyMutexes, maxBodyPairs, maxContacts,
                mapObj2Bp, objVsBpFilter, objVsObjFilter);

        // To enable the callbacks, register this app as a tick listener:
        addTickListener(this);

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    public void initialize() {
        super.initialize();

        setVsync(true);
        configureCamera();
        configureInput();
        setBackgroundColor(Constants.SKY_BLUE);
    }

    /**
     * Populate the PhysicsSystem. Invoked once during initialization.
     */
    @Override
    public void populateSystem() {
        ConstShape ballShape = new SphereShape(ballRadius);
        ballBcs = new BodyCreationSettings()
                .setAllowSleeping(false)
                .setAngularDamping(0.9f)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setPosition(0., 4., 0.)
                .setRestitution(0.1f)
                .setShape(ballShape);
        ballBcs.getMassPropertiesOverride().setMass(1f);

        // Restrict the ball's motion to the X-Y plane:
        ballBcs.setAllowedDofs(EAllowedDofs.TranslationX
                | EAllowedDofs.TranslationY | EAllowedDofs.RotationZ);

        restartSimulation(7);
    }

    /**
     * Advance the physics simulation by the specified amount. Invoked during
     * each update.
     *
     * @param wallClockSeconds the elapsed wall-clock time since the previous
     * invocation of {@code updatePhysics} (in seconds, &ge;0)
     */
    @Override
    public void updatePhysics(float wallClockSeconds) {
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
        if (timeSinceAdded >= addInterval) {
            addBall();
            timeSinceAdded = 0f;
        }
        timeSinceAdded += timeStep;
    }
    // *************************************************************************
    // private methods

    /**
     * Add a dynamic ball to the system.
     */
    private void addBall() {
        BodyInterface bi = physicsSystem.getBodyInterface();
        Body body = bi.createBody(ballBcs);
        bi.addBody(body, EActivation.Activate);
        physicsSystem.optimizeBroadPhase();

        // Apply a random horizontal impulse:
        float xImpulse = 1f - 2f * generator.nextFloat();
        body.addImpulse(new Vec3(xImpulse, 0f, 0f));

        visualizeShape(body);
    }

    /**
     * Configure the camera and CIP during initialization.
     */
    private static void configureCamera() {
        CameraInputProcessor cip = getCameraInputProcessor();
        cip.setRotationMode(RotateMode.DragLMB);
        cip.setMoveSpeed(30f);

        cam.setLocation(0f, -23f, 83f);
    }

    /**
     * Configure keyboard input during initialization.
     */
    private void configureInput() {
        getInputManager().add(new InputProcessor() {
            @Override
            public void onKeyboard(int keyId, boolean isPressed) {
                switch (keyId) {
                    case GLFW.GLFW_KEY_4:
                    case GLFW.GLFW_KEY_F4:
                    case GLFW.GLFW_KEY_KP_4:
                        if (isPressed) {
                            restartSimulation(4);
                        }
                        return;

                    case GLFW.GLFW_KEY_5:
                    case GLFW.GLFW_KEY_F5:
                    case GLFW.GLFW_KEY_KP_5:
                        if (isPressed) {
                            restartSimulation(5);
                        }
                        return;

                    case GLFW.GLFW_KEY_6:
                    case GLFW.GLFW_KEY_F6:
                    case GLFW.GLFW_KEY_KP_6:
                        if (isPressed) {
                            restartSimulation(6);
                        }
                        return;

                    case GLFW.GLFW_KEY_7:
                    case GLFW.GLFW_KEY_F7:
                    case GLFW.GLFW_KEY_KP_7:
                        if (isPressed) {
                            restartSimulation(7);
                        }
                        return;

                    case GLFW.GLFW_KEY_8:
                    case GLFW.GLFW_KEY_F8:
                    case GLFW.GLFW_KEY_KP_8:
                        if (isPressed) {
                            restartSimulation(8);
                        }
                        return;

                    case GLFW.GLFW_KEY_9:
                    case GLFW.GLFW_KEY_F9:
                    case GLFW.GLFW_KEY_KP_9:
                        if (isPressed) {
                            restartSimulation(9);
                        }
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
     * Restart the simulation with the specified number of rows of pins.
     *
     * @param numRows (&ge;4, &le;9)
     */
    private void restartSimulation(int numRows) {
        Validate.inRange(numRows, "number of rows", 4, 9);

        physicsSystem.removeAllBodies();

        float barHalfWidth = 0.3f;
        int lastRow = numRows - 1;
        Vec3 tmpOffset = new Vec3();

        // Add child shapes for the pins:
        float pinHalfHeight = 1f;
        float rootHalf = (float) Math.sqrt(0.5);
        float pinHalfWidth = rootHalf * barHalfWidth;
        ConstShape pinShape = new BoxShape(pinHalfWidth);

        float pinSpacing = 2f * (barHalfWidth + ballRadius);
        float rowSpacing = 2f * pinSpacing;

        StaticCompoundShapeSettings fieldShape
                = new StaticCompoundShapeSettings();
        QuatArg rot45 = Quat.sEulerAngles(0f, 0f, Constants.quarterPi);
        for (int rowIndex = 0; rowIndex < numRows; ++rowIndex) {
            float y = -rowSpacing * rowIndex;
            int numPins = numRows - (rowIndex % 2);
            if (rowIndex == lastRow) {
                numPins += 2;
            }
            for (int pinIndex = 0; pinIndex < numPins; ++pinIndex) {
                float x = pinSpacing * (pinIndex - (numPins - 1) / 2f);
                tmpOffset.set(x, y, 0f);
                fieldShape.addShape(tmpOffset, rot45, pinShape);
            }
        }

        // Add child shapes for the vertical bars:
        float barHalfLength = 0.5f * rowSpacing * (11 - numRows);
        BoxShape barShape = new BoxShape(
                barHalfWidth, barHalfLength, pinHalfHeight);
        int numBars = numRows - (lastRow % 2) + 2;
        float yBar = -rowSpacing * lastRow - barHalfLength;

        for (int barIndex = 0; barIndex < numBars; ++barIndex) {
            float x = pinSpacing * (barIndex - (numBars - 1) / 2f);
            fieldShape.addShape(new Vec3(x, yBar, 0f), new Quat(), barShape);
        }

        // Add a child shape for the horizontal stop at the bottom:
        float yStop = yBar - barHalfLength;
        float stopHalfWidth = pinSpacing * (numBars - 1) / 2f + barHalfWidth;
        BoxShapeSettings stopShape = new BoxShapeSettings(
                stopHalfWidth, barHalfWidth, pinHalfHeight);
        fieldShape.addShape(0f, yStop, 0f, stopShape);

        ConstShape shape = fieldShape.create().get();

        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Static)
                .setObjectLayer(objLayerNonMoving)
                .setRestitution(0.1f)
                .setShape(shape);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body playingField = bi.createBody(bcs);
        bi.addBody(playingField, EActivation.DontActivate);

        visualizeShape(playingField);

        timeSinceAdded = addInterval;
        physicsSystem.optimizeBroadPhase();
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private static void togglePause() {
        float pausedSpeed = 1e-9f;
        physicsSpeed = (physicsSpeed <= pausedSpeed) ? 1f : pausedSpeed;
    }
}
