/*
 Copyright (c) 2025 Stephen Gold and Yanis Boudiaf

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
import com.github.stephengold.joltjni.BoxShapeSettings;
import com.github.stephengold.joltjni.CustomContactListener;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.SphereShapeSettings;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstBodyCreationSettings;
import com.github.stephengold.joltjni.readonly.ConstShapeSettings;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.joltjni.std.DefaultRandomEngine;
import com.github.stephengold.joltjni.std.RandomNumberEngine;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Utils;
import com.github.stephengold.sportjolt.input.CameraInputProcessor;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.RigidBodyShapeGeometry;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import org.joml.Quaternionf;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.lwjgl.glfw.GLFW;

/**
 * A physics-based demo game for SPORT.
 * <p>
 * Adapted from RollingTheMonkey by SkidRunner (Mark E. Picknell).
 * <p>
 * Compare with the original code at
 * https://github.com/jMonkeyEngine/jmonkeyengine/blob/master/jme3-examples/src/main/java/jme3test/games/RollingTheMonkey.java
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class MarbleGame extends BasePhysicsApp {
    // *************************************************************************
    // constants

    /**
     * radius of the marble (in meters)
     */
    final private static float marbleRadius = 2f;
    /**
     * initial number of cubes to collect
     */
    final private static int numCubes = 16;
    // *************************************************************************
    // fields

    /**
     * whether each arrow key is pressed
     */
    final private static boolean[] keyPressed = new boolean[GLFW.GLFW_KEY_LAST];
    /**
     * body virtual addresses of cubes that were collected during the current
     * simulation step
     */
    final private static Collection<Long> newlyCollectedCubes
            = new HashSet<>(numCubes);
    /**
     * maximum force exerted on the marble (in Newtons)
     */
    private static float maxForce;
    /**
     * number of cubes collected
     */
    private static int score = -1;
    /**
     * timestamp of the most recent start (in nanoseconds)
     */
    private static Long startTimestamp;
    /**
     * map virtual addresses of spinning-cube bodies to geometries
     */
    final private static Map<Long, RigidBodyShapeGeometry> uncollectedCubeMap
            = new HashMap<>(numCubes);
    /**
     * geometry that visualizes the marble
     */
    private static RigidBodyShapeGeometry marble;
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the MarbleGame application.
     *
     * @param arguments array of command-line arguments (ignored)
     */
    public static void main(String[] arguments) {
        MarbleGame app = new MarbleGame();
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
        int maxBodies = numCubes + 6;
        int numBpLayers = 2; // use 2 broadphase layers for efficiency
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        result.setContactListener(new CustomContactListener() {
            /**
             * Callback invoked (by native code) each time a new contact point
             * is detected.
             *
             * @param body1Va the virtual address of the first body in contact
             * (not zero)
             * @param body2Va the virtual address of the 2nd body in contact
             * (not zero)
             * @param manifoldVa the virtual address of the contact manifold
             * (not zero)
             * @param settingsVa the virtual address of the contact settings
             * (not zero)
             */
            @Override
            public void onContactAdded(long body1Va, long body2Va,
                    long manifoldVa, long settingsVa) {
                analyzeContact(body1Va, body2Va);
            }
        });

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        System.out.println("Collect the spinning cubes.");
        System.out.println("Use W/A/S/D or arrow keys to move.");
        System.out.println("Press the R key to restart the game.");
        System.out.flush();

        super.initialize();

        setVsync(true);
        configureCamera();
        configureInput();
        configureLighting();
        setBackgroundColor(0.2118f, 0.0824f, 0.6549f, 1f);
    }

    /**
     * Populate the physics system with bodies. Invoked once during
     * initialization.
     */
    @Override
    protected void populateSystem() {
        restartGame();
    }

    /**
     * Advance the physics simulation by the specified interval.
     *
     * @param timeInterval the elapsed (real) time since the previous invocation
     * of {@code updatePhysics} (in seconds, &ge;0)
     */
    @Override
    protected void updatePhysics(float timeInterval) {
        applyForceToMarble();
        spinUncollectedCubes(timeInterval);

        super.updatePhysics(timeInterval);

        if (!newlyCollectedCubes.isEmpty()) {
            removeCollectedCubes();
        }
        reorientCamera();
    }
    // *************************************************************************
    // private methods

    /**
     * Add the yellow cube bodies to the physics system.
     */
    private void addCubes() {
        int seed = (int) System.currentTimeMillis();
        RandomNumberEngine random = new DefaultRandomEngine(seed);

        float cubeHalfExtent = 0.5f; // meters
        ConstShapeSettings cubeShape = new BoxShapeSettings(
                cubeHalfExtent, cubeHalfExtent, cubeHalfExtent);

        float cubeCircleR = 15f; // meters
        Vector3f offset = new Vector3f(0f, 1.5f, -cubeCircleR);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Quaternionf rotation = new Quaternionf();

        for (int cubeIndex = 0; cubeIndex < numCubes; ++cubeIndex) {
            float angle = cubeIndex * Constants.twoPi / numCubes;
            rotation.rotationY(angle);
            Vector3fc location = rotation.transform(offset);
            RVec3Arg joltLocation = Utils.toLocationVector(location);

            QuatArg joltOrientation = Quat.sRandom(random);
            BodyCreationSettings bcs = new BodyCreationSettings()
                    .setIsSensor(true)
                    .setMotionType(EMotionType.Kinematic)
                    .setObjectLayer(objLayerMoving)
                    .setPosition(joltLocation)
                    .setRotation(joltOrientation)
                    .setShapeSettings(cubeShape);

            Body body = bi.createBody(bcs);
            bi.addBody(body, EActivation.Activate);

            RigidBodyShapeGeometry geometry
                    = (RigidBodyShapeGeometry) visualizeShape(body);
            geometry.setProgram("Phong/Distant/Monochrome");

            long bodyVa = body.targetVa();
            uncollectedCubeMap.put(bodyVa, geometry);
        }
    }

    /**
     * Add the green floor body to the physics system.
     */
    private void addFloor() {
        ConstShapeSettings floorShape = new BoxShapeSettings(22f, 0.5f, 22f);
        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Static)
                .setObjectLayer(objLayerNonMoving)
                .setPosition(0f, -0.5f, 0f)
                .setShapeSettings(floorShape);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body body = bi.createBody(bcs);
        bi.addBody(body, EActivation.DontActivate);

        visualizeShape(body)
                .setColor(Constants.GREEN)
                .setProgram("Phong/Distant/Monochrome");
    }

    /**
     * Add the marble body to the physics system.
     */
    private void addMarble() {
        float dropDistance = marbleRadius; // initial drop
        ConstShapeSettings shapeSettings
                = new SphereShapeSettings(marbleRadius);
        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Dynamic)
                .setObjectLayer(objLayerMoving)
                .setPosition(0., dropDistance + marbleRadius, 0.)
                .setRestitution(0.1f)
                .setShapeSettings(shapeSettings);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body body = bi.createBody(bcs);
        bi.addBody(body, EActivation.Activate);

        marble = (RigidBodyShapeGeometry) visualizeShape(body)
                .setColor(Constants.GRAY)
                .setProgram("Phong/Distant/Texture");

        float maxAcceleration = 9f; // meters per second^2
        float invMass = body.getMotionProperties().getInverseMass();
        maxForce = maxAcceleration / invMass;
    }

    /**
     * Create a red wall body using the specified settings, add the body to the
     * physics system, and visualize its shape.
     *
     * @param bcs the settings to use (not null, unaffected)
     */
    private void addRedWall(ConstBodyCreationSettings bcs) {
        BodyInterface bi = physicsSystem.getBodyInterface();
        ConstBody body = bi.createBody(bcs);
        bi.addBody(body, EActivation.DontActivate);

        visualizeShape(body)
                .setColor(Constants.RED)
                .setProgram("Phong/Distant/Monochrome");
    }

    /**
     * Add the 4 wall bodies to the physics system.
     */
    private void addWalls() {
        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Static)
                .setObjectLayer(objLayerNonMoving);

        float halfHeight = 2f;
        float halfThickness = 0.5f;
        float wallCircleR = 21.5f;

        // Shape settings for the east (+X) and west (-X) walls:
        float ewWallHalfLength = wallCircleR - halfThickness;
        ConstShapeSettings ewShape = new BoxShapeSettings(
                halfThickness, halfHeight, ewWallHalfLength);
        bcs.setShapeSettings(ewShape);

        bcs.setPosition(-wallCircleR, halfHeight, 0f);
        addRedWall(bcs);

        bcs.setPosition(wallCircleR, halfHeight, 0f);
        addRedWall(bcs);

        // Shape settings for the north (+Z) and south (-Z) walls:
        float nsWallHalfLength = wallCircleR + halfThickness;
        ConstShapeSettings nsShape = new BoxShapeSettings(
                nsWallHalfLength, halfHeight, halfThickness); // TODO
        bcs.setShapeSettings(nsShape);

        bcs.setPosition(0., halfHeight, wallCircleR);
        addRedWall(bcs);

        bcs.setPosition(0., halfHeight, -wallCircleR);
        addRedWall(bcs);
    }

    /**
     * Check for contact between the marble and an uncollected spinning cube. If
     * such a contact is found, the cube is removed from the map and queued for
     * removal from the physics system.
     *
     * @param va1 the virtual address of the first contact body
     * @param va2 the virtual address of the other contact body
     */
    private void analyzeContact(long va1, long va2) {
        long marbleBodyVa = marble.getBodyVa();

        if (va1 == marbleBodyVa) {
            RigidBodyShapeGeometry geometry = uncollectedCubeMap.get(va2);
            if (geometry != null) {
                uncollectedCubeMap.remove(va2);
                newlyCollectedCubes.add(va2);
            }

        } else if (va2 == marbleBodyVa) {
            RigidBodyShapeGeometry geometry = uncollectedCubeMap.get(va1);
            if (geometry != null) {
                uncollectedCubeMap.remove(va1);
                newlyCollectedCubes.add(va1);
            }
        }
    }

    /**
     * Apply a horizontal force to move the marble, based on which movement keys
     * are pressed.
     */
    private void applyForceToMarble() {
        Vector3f direction = cam.direction(null);
        Vector3f right = cam.rightDirection(null);

        Vector3f force = new Vector3f();
        if (keyPressed[GLFW.GLFW_KEY_DOWN] || keyPressed[GLFW.GLFW_KEY_S]) {
            force.sub(direction);
        }
        if (keyPressed[GLFW.GLFW_KEY_LEFT] || keyPressed[GLFW.GLFW_KEY_A]) {
            force.sub(right);
        }
        if (keyPressed[GLFW.GLFW_KEY_RIGHT] || keyPressed[GLFW.GLFW_KEY_D]) {
            force.add(right);
        }
        if (keyPressed[GLFW.GLFW_KEY_UP] || keyPressed[GLFW.GLFW_KEY_W]) {
            force.add(direction);
        }

        int marbleBodyId = marble.getBodyId();
        if (force.lengthSquared() > 0f) {
            force.y = 0f;    // stop ball from pushing down or flying up
            force.normalize();       // normalize force
            force.mul(maxForce);  // scale vector to force
            Vec3Arg joltForce = Utils.toJoltVector(force);

            BodyInterface bi = physicsSystem.getBodyInterface();
            bi.addForce(marbleBodyId, joltForce);
        }
    }

    /**
     * Configure the Camera and CIP during initialization.
     */
    private static void configureCamera() {
        cam.setLocation(0f, 12f, 21f);

        CameraInputProcessor cip = getCameraInputProcessor();
        cip.setMoveSpeed(0f);
        cip.setRotationMode(RotateMode.None);
    }

    /**
     * Configure keyboard input during initialization.
     */
    private void configureInput() {
        getInputManager().add(new InputProcessor() {
            @Override
            public void onKeyboard(int keyId, boolean isPressed) {
                keyPressed[keyId] = isPressed;
                switch (keyId) {
                    case GLFW.GLFW_KEY_R:
                        if (isPressed) {
                            System.out.println("Restarting the game.");
                            restartGame();
                        }
                        return;

                    default:
                }
                super.onKeyboard(keyId, isPressed);
            }
        });
    }

    /**
     * Configure lighting during initialization.
     */
    private void configureLighting() {
        setLightDirection(0.7683498f, 0.3292928f, 0.54882133f);
        setLightColor(Constants.WHITE);
    }

    /**
     * Remove any collected cubes from the physics system and update the score.
     */
    private void removeCollectedCubes() {
        BodyInterface bi = physicsSystem.getBodyInterface();
        for (long collectedVa : newlyCollectedCubes) {
            Body collectedBody = new Body(collectedVa);
            int collectedBodyId = collectedBody.getId();
            bi.removeBody(collectedBodyId);

            setScore(score + 1);
        }
        newlyCollectedCubes.clear();
        physicsSystem.optimizeBroadPhase();
    }

    /**
     * Turn the camera so it follows the marble.
     */
    private void reorientCamera() {
        BodyInterface bi = physicsSystem.getBodyInterface();
        int marbleBodyId = marble.getBodyId();
        RVec3Arg joltLocation = bi.getPosition(marbleBodyId);

        Vector3fc cameraLocation = cam.location(null);
        Vector3f lookDirection = Utils.toJomlVector(joltLocation);
        lookDirection.sub(cameraLocation);
        cam.setLookDirection(lookDirection);
    }

    /**
     * Restart the game.
     */
    private void restartGame() {
        cleanUp();

        newlyCollectedCubes.clear();
        uncollectedCubeMap.clear();

        addCubes();
        addFloor();
        addMarble();
        addWalls();

        setScore(0);
        startTimestamp = System.nanoTime();
    }

    /**
     * Update the player's score.
     *
     * @param newScore the desired score (in points)
     */
    private static void setScore(int newScore) {
        if (newScore != score) {
            score = newScore;
            System.out.printf("Score:  %d%n", score);
            if (score >= numCubes) {
                long endTimestamp = System.nanoTime();
                long elapsedNanos = endTimestamp - startTimestamp;
                float elapsedSeconds = 1e-9f * elapsedNanos;
                System.out.printf(
                        "You collected all %d cubes in %.1f seconds.%n",
                        numCubes, elapsedSeconds);
                System.out.println("Thanks for playing!");
            }
        }
    }

    /**
     * Kinematically rotate each uncollected cube.
     *
     * @param timeInterval the (real) time interval since the previous physics
     * update (in seconds)
     */
    private void spinUncollectedCubes(float timeInterval) {
        float spinRate = 10f; // radians per second
        float angle = spinRate * timeInterval;
        BodyInterface bi = physicsSystem.getBodyInterface();
        Quaternionf rot = new Quaternionf().rotationAxis(angle, 1f, 0f, 0f);

        for (RigidBodyShapeGeometry cube : uncollectedCubeMap.values()) {
            int bodyId = cube.getBodyId();

            Quat joltOrientation = bi.getRotation(bodyId);
            Quaternionf orientation = Utils.toJomlQuaternion(joltOrientation);
            orientation.premul(rot);
            Utils.toJoltQuaternion(orientation, joltOrientation);

            RVec3Arg joltLocation = bi.getPosition(bodyId);
            bi.moveKinematic(
                    bodyId, joltLocation, joltOrientation, timeInterval);
        }
    }
}
