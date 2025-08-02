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
import com.github.stephengold.joltjni.BodyIdArray;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BoxShape;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Plane;
import com.github.stephengold.joltjni.PlaneShape;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionQuality;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstPlane;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.std.Std;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.TextureKey;
import com.github.stephengold.sportjolt.Utils;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.mesh.CrosshairsMesh;
import com.github.stephengold.sportjolt.mesh.LoopMesh;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import java.util.Random;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;
import org.lwjgl.glfw.GLFW;

/**
 * A 3-D physics demo that drops 1000 cubes onto a horizontal surface and
 * launches balls at them.
 */
public class ThousandCubes extends BasePhysicsApp {
    // *************************************************************************
    // constants

    /**
     * simulation speed when "paused"
     */
    final private static float pausedSpeed = 1e-9f;
    /**
     * how many columns of boxes (along the system's Z axis)
     */
    final private static int numColumns = 10;
    /**
     * how many layers of boxes (along the system's Y axis)
     */
    final private static int numLayers = 10;
    /**
     * how many rows of boxes (along the system's X axis)
     */
    final private static int numRows = 10;
    // *************************************************************************
    // fields

    /**
     * settings for creating boxes
     */
    private static BodyCreationSettings boxBcs;
    /**
     * settings for creating bodies when the E key is pressed
     */
    private static BodyCreationSettings launchBcs;
    /**
     * total time simulated as of the previous window-title update (in seconds)
     */
    private static Double previousTotalSimulatedTime;
    /**
     * simulation speed (simulated seconds per wall-clock second)
     */
    private static float physicsSpeed = 1f;
    /**
     * cross geometry for the crosshairs
     */
    private static Geometry cross;
    /**
     * loop geometry for the crosshairs
     */
    private static Geometry loop;
    /**
     * timestamp of the previous window-title update (in nanoseconds)
     */
    private static Long previousTitleUpdate;
    /**
     * total wall-clock time in physics simulation as of the previous
     * window-title update (in nanoseconds)
     */
    private static Long previousTotalPhysicsNanos;
    /**
     * generate random colors for boxes
     */
    final private static Random random = new Random();
    // *************************************************************************
    // constructors

    /**
     * Instantiate the ThousandCubes application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public ThousandCubes() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the ThousandCubes application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        ThousandCubes app = new ThousandCubes();
        app.start();
    }
    // *************************************************************************
    // BasePhysicsApp methods

    /**
     * Create the physics system. Invoked once during initialization.
     *
     * @return a new object
     */
    @Override
    protected PhysicsSystem createSystem() {
        if (Utils.areAssertionsEnabled()) {
            System.out.println("Warning:  assertions are enabled.");
        }
        if (Jolt.buildType().equals("Debug")) {
            System.out.println("Warning:  using a Debug native library");
        }
        if (Jolt.isDoublePrecision()) {
            System.out.println("Warning:  using a Dp native library");
        }

        int numBoxes = numColumns * numLayers * numRows;
        int maxBodies = numBoxes + 500; // allow for 499 balls plus the floor

        int numBpLayers = 2; // use 2 broadphase layers for efficiency
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        super.initialize();

        setVsync(true);
        addCrosshairs();
        configureCamera();
        configureInput();
        setBackgroundColor(Constants.SKY_BLUE);
    }

    /**
     * Populate the physics system with bodies. Invoked once during
     * initialization.
     */
    @Override
    protected void populateSystem() {
        // Create and add the floor (a static body):
        ConstPlane plane = new Plane(0f, 1f, 0f, 0f);
        ConstShape shape = new PlaneShape(plane);
        BodyCreationSettings floorBcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Static)
                .setObjectLayer(objLayerNonMoving)
                .setShape(shape);
        BodyInterface bi = physicsSystem.getBodyInterface();
        Body floor = bi.createBody(floorBcs);
        bi.addBody(floor, EActivation.DontActivate);

        // Visualize the floor:
        String resourceName = "/Textures/greenTile.png";
        float maxAnisotropy = 16f;
        TextureKey textureKey
                = new TextureKey("classpath://" + resourceName, maxAnisotropy);
        visualizeShape(floor, 0.05f)
                .setSpecularColor(Constants.DARK_GRAY)
                .setTexture(textureKey);

        // Configure the settings for creating spheres:
        ConstShape launchShape = new SphereShape(0.5f);
        launchBcs = new BodyCreationSettings()
                .setAngularDamping(0.1f)
                .setLinearDamping(0.3f)
                .setMotionQuality(EMotionQuality.LinearCast)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setShape(launchShape);
        launchBcs.getMassPropertiesOverride().setMass(10f);

        // Configure the settings for creating boxes:
        ConstShape boxShape = new BoxShape(0.5f);
        boxBcs = new BodyCreationSettings()
                .setAngularDamping(0.1f)
                .setLinearDamping(0.3f)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setShape(boxShape);
        boxBcs.getMassPropertiesOverride().setMass(10f);

        // Create many boxes, arranged in a rectangular grid:
        int numBoxes = numColumns * numLayers * numRows;
        BodyIdArray boxIds = new BodyIdArray(numBoxes);
        int boxIndex = 0;
        for (int i = 0; i < numRows; ++i) {
            for (int j = 0; j < numLayers; ++j) {
                for (int k = 0; k < numColumns; ++k) {
                    float x = 2f * i;
                    float y = 2f * j;
                    float z = 2f * k;
                    assert boxIndex <= numBoxes;
                    addBox(x, y, z, boxIds, boxIndex);
                    ++boxIndex;
                }
            }
        }

        // Add the boxes to the physics system in a single batch:
        long handle = bi.addBodiesPrepare(boxIds, numBoxes);
        bi.addBodiesFinalize(boxIds, numBoxes, handle, EActivation.Activate);
    }

    /**
     * Callback invoked during each iteration of the main update loop.
     */
    @Override
    protected void render() {
        updateScales();
        super.render();
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
        long now = System.nanoTime();

        if (isPaused()) {
            String initialWindowTitle = initialWindowTitle();
            String title = initialWindowTitle + "  *PAUSED*";
            setWindowTitle(title);

        } else if (previousTitleUpdate == null) {
            previousTitleUpdate = now;
            previousTotalPhysicsNanos = totalPhysicsNanos();
            previousTotalSimulatedTime = totalSimulatedTime();

        } else if (now >= previousTitleUpdate + 1_000_000_000L) {
            long wallClockNanos = now - previousTitleUpdate;
            double wallClockInterval = 1e-9 * wallClockNanos;

            // Estimate the fraction of wall-clock time spent in simulation:
            long totalPhysicsNanos = totalPhysicsNanos();
            long physicsNanos = totalPhysicsNanos - previousTotalPhysicsNanos;
            float physicsPercent = (100f * physicsNanos) / wallClockNanos;

            // Estimate the actual simulation speed:
            double totalSimulatedTime = totalSimulatedTime();
            double simulatedTime
                    = totalSimulatedTime - previousTotalSimulatedTime;
            double actualSpeed = simulatedTime / wallClockInterval;

            String initialWindowTitle = initialWindowTitle();
            String title = String.format("%s   speed=%.2f   %.1f%% physics",
                    initialWindowTitle, actualSpeed, physicsPercent);
            setWindowTitle(title);

            previousTitleUpdate = now;
            previousTotalPhysicsNanos = totalPhysicsNanos;
            previousTotalSimulatedTime = totalSimulatedTime;
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add a dynamic box to the specified array, at the specified coordinates.
     *
     * @param x the desired X coordinate (in system coordinates)
     * @param y the desired Y coordinate (in system coordinates)
     * @param z the desired Z coordinate (in system coordinates)
     * @param boxIds the array to add to (not null, modified)
     * @param boxIndex the index of the box in the array (&ge;0)
     */
    private void addBox(
            float x, float y, float z, BodyIdArray boxIds, int boxIndex) {
        // Create a box:
        boxBcs.setPosition(x, y, z);
        BodyInterface bi = physicsSystem.getBodyInterface();
        ConstBody box = bi.createBody(boxBcs);

        int bodyId = box.getId();
        boxIds.set(boxIndex, bodyId);

        // Visualize the box in a random color:
        float red = Std.pow(random.nextFloat(), 2.2f);
        float green = Std.pow(random.nextFloat(), 2.2f);
        float blue = Std.pow(random.nextFloat(), 2.2f);
        float alpha = 1f;
        visualizeShape(box).setColor(new Vector4f(red, green, blue, alpha));
    }

    /**
     * Create geometries to render yellow crosshairs at the center of the
     * window.
     */
    private static void addCrosshairs() {
        float crossWidth = 0.1f;
        Mesh crossMesh = new CrosshairsMesh(crossWidth, crossWidth);
        cross = new Geometry(crossMesh)
                .setColor(Constants.YELLOW)
                .setProgram("Unshaded/Clipspace/Monochrome");

        int numLines = 32;
        float loopRadius = 0.3f * crossWidth;
        Mesh loopMesh = new LoopMesh(numLines, loopRadius, loopRadius);
        loop = new Geometry(loopMesh)
                .setColor(Constants.YELLOW)
                .setProgram("Unshaded/Clipspace/Monochrome");
    }

    /**
     * Configure the Camera and CIP during initialization.
     */
    private static void configureCamera() {
        getCameraInputProcessor().setRotationMode(RotateMode.Immediate);
        cam.setAzimuth(-2.7f)
                .setLocation(60f, 15f, 28f)
                .setUpAngle(-0.25f);
    }

    /**
     * Configure keyboard input during initialization.
     */
    private void configureInput() {
        getInputManager().add(new InputProcessor() {
            @Override
            public void onKeyboard(int keyId, boolean isPressed) {
                switch (keyId) {
                    case GLFW.GLFW_KEY_E:
                        if (isPressed && !isPaused()) {
                            launchRedBall();
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
        if (physicsSpeed <= pausedSpeed) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Launch a single red ball from the camera location.
     */
    private void launchRedBall() {
        RVec3Arg cameraLocation = cam.getLocation().toRVec3();
        launchBcs.setPosition(cameraLocation);

        float speed = 100f;
        Vec3 velocity = cam.getDirection();
        velocity.scaleInPlace(speed);
        launchBcs.setLinearVelocity(velocity);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body missile = bi.createBody(launchBcs);
        bi.addBody(missile, EActivation.Activate);

        // Visualize the ball in red:
        visualizeShape(missile).setColor(Constants.RED);
    }

    /**
     * Toggle the physics simulation: paused/running.
     */
    private static void togglePause() {
        physicsSpeed = isPaused() ? 1f : pausedSpeed;
    }

    /**
     * Scale the crosshair geometries so they will render as an equal-armed
     * cross and a circle, regardless of the window's aspect ratio.
     */
    private static void updateScales() {
        float aspectRatio = aspectRatio();
        float yScale = Math.min(1f, aspectRatio);
        float xScale = yScale / aspectRatio;
        Vector3fc newScale = new Vector3f(xScale, yScale, 1f);

        cross.setScale(newScale);
        loop.setScale(newScale);
    }
}
