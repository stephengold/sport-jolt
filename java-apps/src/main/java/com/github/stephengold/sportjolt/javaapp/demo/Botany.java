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
import com.github.stephengold.joltjni.HeightFieldShape;
import com.github.stephengold.joltjni.HeightFieldShapeSettings;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RodBendTwist;
import com.github.stephengold.joltjni.RodStretchShear;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.ShapeSettings;
import com.github.stephengold.joltjni.SoftBodyCreationSettings;
import com.github.stephengold.joltjni.SoftBodySharedSettings;
import com.github.stephengold.joltjni.SphereShape;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.Vertex;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionQuality;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.operator.Op;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.ConstSoftBodySharedSettings;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.joltjni.std.DefaultRandomEngine;
import com.github.stephengold.joltjni.std.UniformFloatDistribution;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Utils;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.RodsGeometry;
import java.awt.image.BufferedImage;
import java.nio.FloatBuffer;
import org.lwjgl.glfw.GLFW;

/**
 * A Cosserat-rod demo for SPORT.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class Botany extends BasePhysicsApp {
    // *************************************************************************
    // fields

    /**
     * settings for creating a body each time the E key is pressed
     */
    private static BodyCreationSettings launchBcs;
    /**
     * source of (pseudo-)randomness
     */
    private static DefaultRandomEngine random;
    /**
     * shape of the terrain
     */
    private static HeightFieldShape groundShape;
    /**
     * generate single-precision values between 0 and 2 pi
     */
    private static UniformFloatDistribution angleDistribution;
    /**
     * generate single-precision values between 0 and 1
     */
    private static UniformFloatDistribution fractionDistribution;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the Botany application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public Botany() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the Botany application.
     *
     * @param arguments array of command-line arguments (ignored)
     */
    public static void main(String[] arguments) {
        Botany app = new Botany();
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
        int maxBodies = 200; // for balls, mostly
        int numBpLayers = 2; // use 2 broadphase layers for efficiency
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        return result;
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        // Intialize the randomness:
        random = new DefaultRandomEngine();
        angleDistribution = new UniformFloatDistribution(0f, Constants.twoPi);
        fractionDistribution = new UniformFloatDistribution(0f, 1f);

        super.initialize();

        setVsync(true);
        configureCamera();
        configureInput();
        configureLighting();
        setBackgroundColor(Constants.SKY_BLUE);

        // Intialize the settings for creating balls:
        ConstShape launchShape = new SphereShape(1f);
        launchBcs = new BodyCreationSettings()
                .setAngularDamping(0.3f)
                .setLinearDamping(0.3f)
                .setMotionQuality(EMotionQuality.LinearCast)
                .setOverrideMassProperties(
                        EOverrideMassProperties.CalculateInertia)
                .setShape(launchShape);
        launchBcs.getMassPropertiesOverride().setMass(50f);
    }

    /**
     * Populate the physics system with bodies. Invoked once during
     * initialization.
     */
    @Override
    protected void populateSystem() {
        ConstBody ground = addTerrain();
        visualizeShape(ground)
                .setSpecularColor(Constants.BLACK);

        // Add trees and tussocks in a perturbed grid:
        for (int i = 0; i < 6; ++i) {
            float z = -8f * i;
            for (int j = 0; j < 6; ++j) {
                float x = -8f * j;

                float p = fractionDistribution.nextFloat(random);
                if (p > 0.8f) { // Place a tree:
                    ConstSoftBodySharedSettings settings = generateTree();
                    float damping = 0.1f;
                    ConstBody tree = addSoftBody(settings, x, z, damping);
                    new RodsGeometry(tree)
                            .setColor(0.1f, 0.4f, 0f);

                } else if (p < 0.3f) { // Place a tussock:
                    ConstSoftBodySharedSettings settings = generateTussock();
                    float damping = 9f;
                    ConstBody tussock = addSoftBody(settings, x, z, damping);
                    new RodsGeometry(tussock)
                            .setColor(0.5f, 1f, 0.2f);
                }
            }
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Recursively add vertices and rods to the specified shared settings to
     * simulate a tree.
     *
     * @param sbss the settings to append to (not null, modified)
     * @param startVertex the index of the start vertex
     * @param previousRod the index of the previous rod, or -1 if none
     * @param offset the initial length and direction of the new rod (not null,
     * unaffected)
     * @param level the depth of the recursion
     */
    private static void addLimbAndFork(SoftBodySharedSettings sbss,
            int startVertex, int previousRod, Vec3Arg offset, int level) {

        // Configure and add the fork vertex:
        Vertex previousVertex = sbss.getVertex(startVertex);
        float invMass = previousVertex.getInvMass();
        invMass = (invMass > 0f) ? 3f * invMass : 1e-3f;

        Vec3 location = new Vec3(previousVertex.getPosition());
        location.addInPlace(
                offset.getX(), offset.getY(), offset.getZ());

        Vertex vertex = new Vertex()
                .setInvMass(invMass)
                .setPosition(location);
        int endVertex = sbss.countVertices();
        sbss.addVertex(vertex);

        // Configure and add the rod:
        int rodIndex = sbss.countRodStretchShearConstraints();
        RodStretchShear rod = new RodStretchShear(startVertex, endVertex);
        sbss.addRodStretchShearConstraint(rod);
        if (previousRod != -1) {
            RodBendTwist constraint
                    = new RodBendTwist(previousRod, rodIndex);
            sbss.addRodBendTwistConstraint(constraint);
        }

        // Create sub branches:
        if (level < 10) {
            int nextLevel = level + 1;
            float theta = angleDistribution.nextFloat(random);
            for (int i = 0; i < 2; ++i) {
                // Choose a direction:
                float axisX = Jolt.sin(theta);
                float axisZ = Jolt.cos(theta);
                Vec3 axis = new Vec3(axisX, 0f, axisZ);
                float extra = 0.1f * fractionDistribution.nextFloat(random);
                float bendAngle = 0.1f * level + extra;
                Vec3 nextOffset
                        = Op.star(Quat.sRotation(axis, bendAngle), offset);
                nextOffset.scaleInPlace(0.9f);

                addLimbAndFork(
                        sbss, endVertex, rodIndex, nextOffset, nextLevel);
                theta += Math.PI;
            }
        }
    }

    /**
     * Add a soft body to the scene at the specified location.
     *
     * @param settings the shared settings to use (not null, unaffected)
     * @param x the approximate desired X coordinate for the root vertex
     * @param z the approximate desired Z coordinate for the root vertex
     * @param damping the desired damping constant
     * @return the new body
     */
    private ConstBody addSoftBody(ConstSoftBodySharedSettings settings,
            float x, float z, float damping) {

        // Perturb the desired location and project it onto the heighfield:
        float dx = fractionDistribution.nextFloat(random)
                - fractionDistribution.nextFloat(random);
        float dz = fractionDistribution.nextFloat(random)
                - fractionDistribution.nextFloat(random);
        Vec3Arg location = new Vec3(x + dx, 0., z + dz);
        Vec3 storeLocation = new Vec3();
        groundShape.projectOntoSurface(location, storeLocation, new int[1]);

        SoftBodyCreationSettings sbcs = new SoftBodyCreationSettings()
                .setLinearDamping(damping)
                .setPosition(storeLocation.toRVec3())
                .setSettings(settings);
        BodyInterface bi = physicsSystem.getBodyInterface();
        ConstBody result = bi.createSoftBody(sbcs);
        bi.addBody(result, EActivation.Activate);

        return result;
    }

    /**
     * Add a static heightfield rigid body to the system.
     *
     * @return the new body (not null)
     */
    private ConstBody addTerrain() {
        // Generate an array of heights from a PNG image on the classpath:
        String resourceName = "/Textures/Terrain/height/basin.png";
        BufferedImage image = Utils.loadResourceAsImage(resourceName);

        float maxHeight = 51f;
        FloatBuffer heightBuffer = Utils.toHeightBuffer(image, maxHeight);

        // Construct a static rigid body based on the array of heights:
        int numFloats = heightBuffer.capacity();

        Vec3Arg offset = new Vec3(-256f, 0f, -256f);
        Vec3Arg scale = new Vec3(1f, 1f, 1f);
        int sampleCount = 512;
        assert numFloats == sampleCount * sampleCount : numFloats;
        ShapeSettings ss = new HeightFieldShapeSettings(
                heightBuffer, offset, scale, sampleCount);

        ShapeRefC shapeRef = ss.create().get();
        groundShape = (HeightFieldShape) shapeRef.getPtr();
        BodyCreationSettings bcs = new BodyCreationSettings()
                .setMotionType(EMotionType.Static)
                .setObjectLayer(objLayerNonMoving)
                .setShape(groundShape);

        BodyInterface bi = physicsSystem.getBodyInterface();
        ConstBody result = bi.createBody(bcs);
        bi.addBody(result, EActivation.DontActivate);

        return result;
    }

    /**
     * Configure the camera and CIP during initialization.
     */
    private static void configureCamera() {
        getCameraInputProcessor()
                .setMoveSpeed(20f)
                .setRotationMode(RotateMode.Immediate);
        cam.setAzimuth(-0.8f)
                .setLocation(-51f, 10f, 16f)
                .setUpAngle(-0.12f);
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
                        if (isPressed) {
                            launchRedBall();
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
    }

    /**
     * Generate shared settings for springy trees.
     *
     * @return a new object
     */
    private static SoftBodySharedSettings generateTree() {
        SoftBodySharedSettings result = new SoftBodySharedSettings();

        // Begin with the pinned root vertex:
        Vertex rootVertex = new Vertex().setInvMass(0f);
        result.addVertex(rootVertex);

        int startVertex = 0;
        int previousRod = -1;
        Vec3Arg trunkOffset = new Vec3(0f, 2f, 0f);
        int level = 0;
        addLimbAndFork(result, startVertex, previousRod, trunkOffset, level);

        result.calculateRodProperties();
        result.optimize();

        return result;
    }

    /**
     * Generate shared settings for tussocks with spiral stalks.
     *
     * @return a new object
     */
    private static SoftBodySharedSettings generateTussock() {
        int numVerticesPerStalk = 64;
        float discRadius = 0.5f; // meters
        float spiralRadius = 0.1f; // meters

        SoftBodySharedSettings result = new SoftBodySharedSettings();
        int numStalks = 70;
        for (int stalk = 0; stalk < numStalks; ++stalk) {
            int vertex0 = result.countVertices();
            int rod0 = result.countRodStretchShearConstraints();

            // Randomly place the stalk's root within a horizontal disc:
            float r = discRadius * fractionDistribution.nextFloat(random);
            float theta = angleDistribution.nextFloat(random);
            float x0 = r * Jolt.sin(theta);
            float z0 = r * Jolt.cos(theta);

            // Randomize the starting phase of the spiral:
            float phase0 = angleDistribution.nextFloat(random);

            // Make the outer stalks longer than the inner ones.
            float rodDeltaY = 0.05f + 0.05f * r; // meters

            // The first (root) vertex of each stalk is pinned:
            Vertex vertex = new Vertex().setInvMass(0f);

            for (int i = 0; i < numVerticesPerStalk; ++i) {
                float y = rodDeltaY * i;

                float phase = phase0 + 0.5f * i;
                float x = spiralRadius * Jolt.sin(phase) + (y + 1f) * x0;
                float z = spiralRadius * Jolt.cos(phase) + (y + 1f) * z0;
                vertex.setPosition(x, y, z);
                result.addVertex(vertex);

                // Successive stalks are movable:
                vertex.setInvMass(10f);
            }

            // Join each pair of adjacent vertices with a rod:
            RodStretchShear rod = new RodStretchShear();
            for (int i = 0; i < numVerticesPerStalk - 1; ++i) {
                int vi = vertex0 + i;
                rod.setVertex(0, vi);
                rod.setVertex(1, vi + 1);
                result.addRodStretchShearConstraint(rod);
            }

            // Join each pair of adjacent rods with a bend-twist constraint:
            RodBendTwist constraint = new RodBendTwist();
            for (int i = 0; i < numVerticesPerStalk - 2; ++i) {
                int ri = rod0 + i;
                constraint.setRod(0, ri);
                constraint.setRod(1, ri + 1);
                result.addRodBendTwistConstraint(constraint);
            }
        }

        result.calculateRodProperties();
        result.optimize();

        return result;
    }

    /**
     * Launch a single red ball from the camera location.
     */
    private void launchRedBall() {
        RVec3Arg cameraLocation = cam.getLocation().toRVec3();
        launchBcs.setPosition(cameraLocation);

        float speed = 200f; // meters per second
        Vec3 velocity = cam.getDirection();
        velocity.scaleInPlace(speed);
        launchBcs.setLinearVelocity(velocity);

        BodyInterface bi = physicsSystem.getBodyInterface();
        Body missile = bi.createBody(launchBcs);
        bi.addBody(missile, EActivation.Activate);

        // Visualize the ball in red:
        visualizeShape(missile).setColor(Constants.RED);
    }
}
