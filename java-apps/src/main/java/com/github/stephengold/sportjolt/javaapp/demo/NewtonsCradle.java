/*
 Copyright (c) 2020-2025 Stephen Gold and Yanis Boudiaf

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
import com.github.stephengold.joltjni.DistanceConstraintSettings;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.TwoBodyConstraint;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.input.CameraInputProcessor;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.ConstraintGeometry;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.joml.Vector4f;
import org.joml.Vector4fc;
import org.lwjgl.glfw.GLFW;

/**
 * A physics demo that simulates a "Newton's cradle" toy.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Newton%27s_cradle">Wikipedia</a>
 * @author Stephen Gold sgold@sonic.net
 */
public class NewtonsCradle extends BasePhysicsApp {
    // *************************************************************************
    // constants

    /**
     * simulation speed when "paused"
     */
    final private static float PAUSED_SPEED = 1e-9f;
    /**
     * color to visualize balls: nearly black
     */
    final private static Vector4fc BALL_COLOR
            = new Vector4f(0.01f, 0.01f, 0.01f, 1f);
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
    // *************************************************************************
    // constructors

    /**
     * Instantiate the NewtonsCradle application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public NewtonsCradle() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the NewtonsCradle application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Logger.getLogger("").setLevel(Level.WARNING);
        NewtonsCradle application = new NewtonsCradle();
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
        int maxBodies = 5; // allow for 5 balls
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        result.setGravity(0f, -150f, 0f);

        // Reduce the timestep for better accuracy:
        this.timePerStep = 0.01f;

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        super.initialize();

        setVsync(true);
        configureCamera();
        configureInput();
        setBackgroundColor(Constants.SKY_BLUE);
    }

    /**
     * Populate the PhysicsSystem with bodies and constraints. Invoked once
     * during initialization.
     */
    @Override
    protected void populateSystem() {
        float radius = 9.9f;
        ConstShape shape = new SphereShape(radius);
        ballBcs = new BodyCreationSettings()
                .setFriction(0f)
                .setRestitution(1f)
                .setShape(shape);

        restartSimulation(5);
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

    /**
     * Invoked before each frame is rendered, to update the text in the window's
     * title bar.
     */
    @Override
    protected void updateWindowTitle() {
        String initialWindowTitle = initialWindowTitle();
        String title;
        if (isPaused()) {
            title = initialWindowTitle + "  *PAUSED*";
        } else {
            title = initialWindowTitle;
        }
        setWindowTitle(title);
    }
    // *************************************************************************
    // private methods

    /**
     * Add a dynamic ball to the system, suspended between 2 single-ended
     * distance constraints.
     *
     * @param xOffset the desired location along the system's X axis
     * @return a new instance
     */
    private Body addSuspendedBall(float xOffset) {
        // Create and add the ball:
        ballBcs.setPosition(xOffset, 0., 0.);
        BodyInterface bi = physicsSystem.getBodyInterface();
        Body result = bi.createBody(ballBcs);
        int bodyId = result.getId();
        bi.addBody(bodyId, EActivation.Activate);

        // Visualize the ball:
        visualizeShape(result).setColor(BALL_COLOR);

        DistanceConstraintSettings dcs = new DistanceConstraintSettings();
        float wireLength = 80f;
        dcs.setMaxDistance(wireLength);
        dcs.setMinDistance(0f);
        float yOffset = wireLength * Constants.rootHalf;

        // Create and add the constraint on the +Z side:
        dcs.setPoint1(new RVec3(xOffset, 0f, 0f));
        dcs.setPoint2(new RVec3(xOffset, yOffset, +yOffset));
        TwoBodyConstraint joint1 = dcs.create(result, Body.sFixedToWorld());
        physicsSystem.addConstraint(joint1);
        new ConstraintGeometry(joint1, 0);

        // Create and add the constraint on the -Z side:
        dcs.setPoint1(new RVec3(xOffset, 0f, 0f));
        dcs.setPoint2(new RVec3(xOffset, yOffset, -yOffset));
        TwoBodyConstraint joint2 = dcs.create(result, Body.sFixedToWorld());
        physicsSystem.addConstraint(joint2);
        new ConstraintGeometry(joint2, 0);

        return result;
    }

    /**
     * Configure the Camera and CIP during initialization.
     */
    private static void configureCamera() {
        CameraInputProcessor cip = getCameraInputProcessor();
        cip.setMoveSpeed(30f);
        cip.setRotationMode(RotateMode.DragLMB);

        cam.setAzimuth(-2f)
                .setLocation(72f, 35f, 140f)
                .setUpAngle(-0.2f);
    }

    /**
     * Configure keyboard input during initialization.
     */
    private void configureInput() {
        getInputManager().add(new InputProcessor() {
            @Override
            public void onKeyboard(int keyId, boolean isPressed) {
                switch (keyId) {
                    case GLFW.GLFW_KEY_1:
                    case GLFW.GLFW_KEY_F1:
                    case GLFW.GLFW_KEY_KP_1:
                        if (isPressed) {
                            restartSimulation(1);
                        }
                        return;

                    case GLFW.GLFW_KEY_2:
                    case GLFW.GLFW_KEY_F2:
                    case GLFW.GLFW_KEY_KP_2:
                        if (isPressed) {
                            restartSimulation(2);
                        }
                        return;

                    case GLFW.GLFW_KEY_3:
                    case GLFW.GLFW_KEY_F3:
                    case GLFW.GLFW_KEY_KP_3:
                        if (isPressed) {
                            restartSimulation(3);
                        }
                        return;

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
     * Test whether physics simulation is paused.
     *
     * @return {@code true} if paused, otherwise {@code false}
     */
    private static boolean isPaused() {
        if (physicsSpeed <= PAUSED_SPEED) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Restart the simulation (paused) with the specified number of balls.
     *
     * @param numBalls (&ge;1)
     */
    private void restartSimulation(int numBalls) {
        Validate.positive(numBalls, "number of balls");

        cleanUp();
        physicsSpeed = PAUSED_SPEED;

        float xSeparation = 20f; // slightly more than the ball diameter

        // center-to-center separation between the first and last balls:
        float xExtent = (numBalls - 1) * xSeparation;

        float x0 = -xExtent / 2;
        Body[] balls = new Body[numBalls];
        for (int ballIndex = 0; ballIndex < numBalls; ++ballIndex) {
            float x = x0 + ballIndex * xSeparation;
            balls[ballIndex] = addSuspendedBall(x);
        }
        physicsSystem.optimizeBroadPhase();

        Vec3Arg kick = new Vec3(-20f * numBalls, 0f, 0f);
        balls[0].setLinearVelocity(kick);
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private static void togglePause() {
        physicsSpeed = isPaused() ? 1f : PAUSED_SPEED;
    }
}
