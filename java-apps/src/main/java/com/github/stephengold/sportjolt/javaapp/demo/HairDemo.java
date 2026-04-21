/*
 Copyright (c) 2026 Stephen Gold and Yanis Boudiaf

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

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.ComputeQueue;
import com.github.stephengold.joltjni.ComputeSystem;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.Float3;
import com.github.stephengold.joltjni.Gradient;
import com.github.stephengold.joltjni.Hair;
import com.github.stephengold.joltjni.HairMaterial;
import com.github.stephengold.joltjni.HairSettings;
import com.github.stephengold.joltjni.HairShaders;
import com.github.stephengold.joltjni.HairSkinWeight;
import com.github.stephengold.joltjni.IndexedTriangleNoMaterial;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.Mat44;
import com.github.stephengold.joltjni.Mat44Array;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.SStrand;
import com.github.stephengold.joltjni.SStrandList;
import com.github.stephengold.joltjni.SVertex;
import com.github.stephengold.joltjni.SVertexList;
import com.github.stephengold.joltjni.StreamInWrapper;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.operator.Op;
import com.github.stephengold.joltjni.readonly.ConstFloat3;
import com.github.stephengold.joltjni.readonly.Mat44Arg;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.joltjni.std.Std;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Utils;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.PhysicsTickListener;
import com.github.stephengold.sportjolt.physics.StrandsGeometry;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * A hair-simulation demo for Sport Jolt.
 * <p>
 * Compare with the original by Jorrit Rouwe at
 * https://github.com/jrouwe/JoltPhysics/blob/master/Samples/Tests/Hair/HairTest.cpp
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class HairDemo extends BasePhysicsApp
        implements PhysicsTickListener {
    // *************************************************************************
    // fields

    /**
     * initial locations of hair points
     */
    private static ConstFloat3[] hairPoints;
    /**
     * vertices in the animated mesh
     */
    private static Float3[] animMeshVertices;
    /**
     * skinning weights in the animated mesh
     */
    private static HairSkinWeight[] animMeshWeights;
    /**
     * indices in the animated mesh
     */
    private static IndexedTriangleNoMaterial[] animMeshIndices;
    /**
     * number of joints in the animated skeleton
     */
    private static int numAnimJoints;
    /**
     * number of skinning weights for each vertex in the animated mesh
     */
    private static int numAnimWeightsPerVertex;
    /**
     * bind pose of each joint in the animated skeleton
     */
    private static Mat44[] faceInvBindPose;
    /**
     * hair simulation
     */
    private static Hair hair;
    /**
     * compute shaders used for hair simulation
     */
    private static HairShaders shaders;
    /**
     * current frame of the skeletal animation
     */
    private static int animationFrame;
    /**
     * index of the neck joint in the animated skeleton
     */
    private static int neckJointIndex;
    /**
     * number of points per strand in the hair simulation
     */
    private static int numPointsPerStrand;
    /**
     * number of strands in the hair simulation
     */
    private static int numStrands;
    /**
     * ID of each rigid body attached to the skeleton --- parallel with
     * {@code attachedBodyJointIndices}
     */
    final private static List<Integer> attachedBodyIds = new ArrayList<>();
    /**
     * animation-joint index of each rigid body attached to the skeleton ---
     * parallel with {@code attachedBodyIds}
     */
    final private static List<Integer> attachedBodyJointIndices
            = new ArrayList<>();
    /**
     * key frames of the skeletal animation
     */
    private static Mat44Array[] faceAnimation;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the HairDemo application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public HairDemo() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the HairDemo application.
     *
     * @param arguments array of command-line arguments (ignored)
     */
    public static void main(String[] arguments) {
        HairDemo app = new HairDemo();
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
    public PhysicsSystem createSystem() {
        if (Utils.areAssertionsEnabled()) {
            System.out.println("Warning:  assertions are enabled.");
        }
        if (Jolt.buildType().equals("Debug")) {
            System.out.println("Warning:  using a Debug native library");
        }
        if (Jolt.isDoublePrecision()) {
            System.out.println("Warning:  using a Dp native library");
        }

        int maxBodies = 24;
        int numBpLayers = 2; // use 2 broadphase layers for efficiency
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        // To enable the callbacks, register the application as a tick listener:
        addTickListener(this);

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
        setBackgroundColor(Constants.SKY_BLUE);
    }

    /**
     * Populate the physics system with bodies. Invoked once during
     * initialization.
     */
    @Override
    protected void populateSystem() {
        readAnimMeshFile("Assets/face.bin");

        // Transform the face mesh so it's relative to the neck's bind pose:
        Mat44Arg neckInvBindPose = faceInvBindPose[neckJointIndex];
        Mat44Arg neckBindPose = neckInvBindPose.inversed();
        for (int i = 0; i < animMeshVertices.length; ++i) {
            Op.star(neckInvBindPose, animMeshVertices[i])
                    .storeFloat3(animMeshVertices[i]);
        }
        for (int jointI = 0; jointI < numAnimJoints; ++jointI) {
            faceInvBindPose[jointI].rightMultiplyInPlace(neckBindPose);
        }

        // Read hair file
        readHairModelFile("Assets/wStraight.hair");

        // Init strands
        SStrandList simStrands = new SStrandList();
        SVertexList simVertices = new SVertexList();

        Mat44Arg neckTransform = faceAnimation[0].get(neckJointIndex);
        Mat44 neckInvTransform = neckTransform.inversed();

        int point0 = 0;
        numStrands = Math.min(numStrands, 9_999);
        for (int strandI = 0; strandI < numStrands; ++strandI) {
            // Transform points relative to the neck:
            Vec3[] strandPoints = new Vec3[numPointsPerStrand];
            for (int pointI = 0; pointI < numPointsPerStrand; ++pointI) {
                Vec3 point = new Vec3(hairPoints[point0 + pointI]);
                point.swizzleInPlace(
                        Jolt.SWIZZLE_Y, Jolt.SWIZZLE_Z, Jolt.SWIZZLE_X);
                point.scaleInPlace(0.00254f); // tenths of an inch to meters
                strandPoints[pointI] = Op.star(neckInvTransform, point);
            }

            // Add vertices to the vertex list:
            int startVertex = simVertices.size();
            for (int pointI = 0; pointI < numPointsPerStrand; ++pointI) {
                Vec3Arg initLocation = strandPoints[pointI];
                float invMass = (pointI == 0) ? 0f : 1f;
                SVertex v = new SVertex(initLocation, invMass);
                simVertices.pushBack(v);
            }
            point0 += numPointsPerStrand;

            // Create a new strand and add it to the strand list:
            SStrand simStrand = new SStrand(startVertex, simVertices.size(), 0);
            simStrands.pushBack(simStrand);
        }

        // Resample vertices and strands:
        int verticesPerStrand = 32;
        HairSettings.sResample(simVertices, simStrands, verticesPerStrand);

        // Load shaders:
        ComputeSystem computeSystem = getComputeSystem();
        shaders = new HairShaders().init(computeSystem);

        Vec3Arg worldGravity = physicsSystem.getGravity();
        Vec3Arg gravity = neckInvBindPose.multiply3x3(worldGravity);
        HairSettings hairSettings = createHairSettings(gravity);

        hairSettings.initRenderAndSimulationStrands(simVertices, simStrands);
        float[] maxDistanceSquared = {0.0f};
        hairSettings.init(maxDistanceSquared);
        assert maxDistanceSquared[0] < 1.0e-4f : maxDistanceSquared[0];

        hairSettings.initCompute(computeSystem);

        // Create the hair simulation:
        RVec3 neckLocation = new RVec3(neckTransform.getTranslation());
        Quat neckOrientation = neckTransform.getQuaternion();
        hair = new Hair(
                hairSettings, neckLocation, neckOrientation, objLayerMoving);
        hair.init(computeSystem);

        Mat44Array joints = faceAnimation[animationFrame];
        ComputeQueue queue = getComputeQueue();
        hair.update(0f, neckInvTransform, joints, physicsSystem, shaders,
                computeSystem, queue);
        hair.readBackGpuState(queue);

        // Visualize the strands:
        new StrandsGeometry(hair).setColor(0.2f, 0.2f, 0f);
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback invoked (by Sport-Jolt, not by Jolt Physics) after the system
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
     * Callback invoked (by Sport-Jolt, not by Jolt Physics) before the system
     * is stepped.
     *
     * @param system the system that's about to be stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    public void prePhysicsTick(PhysicsSystem system, float timeStep) {
        // Advance the animation by one frame:
        animationFrame = (animationFrame + 1) % faceAnimation.length;

        // Animate the attached rigid bodies:
        Mat44Array joints = faceAnimation[animationFrame];
        BodyInterface bi = physicsSystem.getBodyInterfaceNoLock();
        for (int abIndex = 0; abIndex < attachedBodyIds.size(); ++abIndex) {
            int bodyId = attachedBodyIds.get(abIndex);
            int jointIndex = attachedBodyJointIndices.get(abIndex);

            RVec3Arg location;
            QuatArg rotation;
            if (jointIndex == -1) {
                location = new RVec3();
                rotation = new Quat();
            } else {
                Mat44Arg transform = joints.get(jointIndex);
                location = new RVec3(transform.getTranslation());
                rotation = transform.getQuaternion();
            }
            bi.moveKinematic(bodyId, location, rotation, timeStep);
        }

        // Reposition the hair:
        Mat44 neckTransform = joints.get(neckJointIndex);
        RVec3 neckLocation = new RVec3(neckTransform.getTranslation());
        hair.setPosition(neckLocation);
        QuatArg neckRotation = neckTransform.getQuaternion();
        hair.setRotation(neckRotation);

        // Simulate the hair:
        ComputeQueue queue = getComputeQueue();
        Mat44 neckInvTransform = neckTransform.inversed();
        hair.update(timeStep, neckInvTransform, joints, physicsSystem,
                shaders, getComputeSystem(), queue);
        queue.executeAndWait();
        hair.readBackGpuState(queue);
    }
    // *************************************************************************
    // new private methods

    /**
     * Configure the camera, projection, and CIP during initialization.
     */
    private static void configureCamera() {
        getCameraInputProcessor()
                .setMoveSpeed(1f)
                .setRotationMode(RotateMode.DragLMB);
        cam.setAzimuth(-0.86f)
                .setLocation(-0.54f, 0.01f, 0.55f)
                .setUpAngle(-0.09f);
        getProjection()
                .setFovy(0.79f)
                .setZNear(0.1f);
    }

    /**
     * Create and configure settings for the hair simulation.
     *
     * @param neckGravity the gravity vector in simulation space (not
     * {@code null}, unaffected)
     * @return a new object
     */
    private HairSettings createHairSettings(Vec3Arg neckGravity) {
        // Configure the hair settings:
        HairMaterial material = new HairMaterial()
                .setAngularDamping(2f)
                .setBendCompliance(1e-7f)
                .setEnableCollision(true)
                .setEnableLra(true)
                .setFriction(0.2f)
                .setLinearDamping(2f)
                .setMaxAngularVelocity(50f)
                .setMaxLinearVelocity(10f)
                .setGlobalPose(new Gradient(0.01f, 0f, 0f, 0.3f))
                .setGravityFactor(new Gradient(0.1f, 1f, 0.2f, 0.8f))
                .setGravityPreloadFactor(1f)
                .setGridDensityForceFactor(0f)
                .setGridVelocityFactor(new Gradient(0.05f, 0.01f))
                .setHairRadius(new Gradient(0.001f, 0.001f))
                .setInertiaMultiplier(10f)
                .setMaxLinearVelocity(10f)
                .setSimulationStrandsFraction(0.1f)
                .setSkinGlobalPose(new Gradient(1f, 0f, 0f, 0.1f))
                .setStretchCompliance(1e-8f)
                .setWorldTransformInfluence(new Gradient(0f, 1f));

        int iterationsPerSecond = HairSettings.cDefaultIterationsPerSecond;
        HairSettings result = new HairSettings()
                .addMaterial(material)
                .setInitialGravity(neckGravity)
                .setNumIterationsPerSecond(iterationsPerSecond)
                .setScalpInverseBindPose(faceInvBindPose)
                .setScalpNumSkinWeightsPerVertex(numAnimWeightsPerVertex)
                .setScalpSkinWeights(animMeshWeights)
                .setScalpTriangles(animMeshIndices)
                .setScalpVertices(animMeshVertices)
                .setSimulationBoundsPadding(Vec3.sReplicate(0.1f));

        return result;
    }

    /**
     * Read an animated 3-D mesh from the specified binary file and generate an
     * attached rigid body for each joint.
     *
     * @param filePath filesystem path to the file (not {@code null})
     */
    private void readAnimMeshFile(String filePath) {
        // Read face mesh, animation, and collision hulls:
        int streamMode = StreamInWrapper.in() | StreamInWrapper.binary();
        StreamInWrapper stream = StreamInWrapper.open(filePath, streamMode);
        neckJointIndex = stream.readInt();

        int numFaceMeshVertices = stream.readInt();
        animMeshVertices = new Float3[numFaceMeshVertices];
        stream.readFloat3Array(animMeshVertices);

        int numFaceMeshIndices = stream.readInt();
        animMeshIndices = new IndexedTriangleNoMaterial[numFaceMeshIndices];
        stream.readIndexedTriangles(animMeshIndices);

        numAnimJoints = stream.readInt();
        faceInvBindPose = new Mat44[numAnimJoints];
        stream.readMatrices(faceInvBindPose);

        numAnimWeightsPerVertex = stream.readInt();
        int numSkinWeights = numAnimWeightsPerVertex * numFaceMeshVertices;
        animMeshWeights = new HairSkinWeight[numSkinWeights];
        stream.readHairSkinWeights(animMeshWeights);

        int numAnimationFrames = stream.readInt();
        faceAnimation = new Mat44Array[numAnimationFrames];
        Float3 tmpRotXyz = new Float3();
        Float3 tmpTranslation = new Float3();
        Quat tmpRotation = new Quat();
        for (int frameI = 0; frameI < numAnimationFrames; ++frameI) {
            faceAnimation[frameI] = new Mat44Array(numAnimJoints);
            for (int jointI = 0; jointI < numAnimJoints; ++jointI) {
                stream.readFloat3(tmpTranslation);
                stream.readFloat3(tmpRotXyz);
                float sumSq = new Vec3(tmpRotXyz).lengthSq();
                float w = Std.sqrt(Math.max(0f, 1f - sumSq));
                tmpRotation.set(tmpRotXyz.x, tmpRotXyz.y, tmpRotXyz.z, w);
                Mat44 transform = Mat44.sRotationTranslation(
                        tmpRotation, new Vec3(tmpTranslation));
                faceAnimation[frameI].set(jointI, transform);
            }
        }

        int numCollisionHulls = stream.readInt();
        Vec3 tmpHullVertex = new Vec3();
        for (int hullI = 0; hullI < numCollisionHulls; ++hullI) {
            int attachJointIndex = stream.readInt();
            int numHullVertices = stream.readInt();

            // Create and add an attached body for the hull:
            ConvexHullShapeSettings hullSettings
                    = new ConvexHullShapeSettings();
            hullSettings.setEmbedded();
            for (int vertexI = 0; vertexI < numHullVertices; ++vertexI) {
                stream.readVec3(tmpHullVertex);
                hullSettings.addPoint(tmpHullVertex);
            }

            Quat initOrientation;
            RVec3 initLocation;
            if (attachJointIndex == -1) {
                initLocation = new RVec3();
                initOrientation = new Quat();
            } else {
                Mat44 ajMatrix = faceAnimation[0].get(attachJointIndex);
                initLocation = new RVec3(ajMatrix.getTranslation());
                initOrientation = ajMatrix.getQuaternion();
                Mat44 ajInverseMatrix = ajMatrix.inversed();
                hullSettings.transformPoints(ajInverseMatrix);
            }

            BodyInterface bi = physicsSystem.getBodyInterface();
            BodyCreationSettings body = new BodyCreationSettings(
                    hullSettings, initLocation, initOrientation,
                    EMotionType.Kinematic, objLayerMoving);
            int bodyId = bi.createAndAddBody(body, EActivation.DontActivate);
            visualizeBodyShape(bodyId);
            attachedBodyIds.add(bodyId);
            attachedBodyJointIndices.add(attachJointIndex);
        }
    }

    /**
     * Read a hair model from the specified binary file.
     *
     * @param filePath filesystem path to the file (not {@code null})
     */
    private static void readHairModelFile(String filePath) {
        ByteBuffer data = Utils.loadFileAsBytes(filePath);
        assert data.get(0) == 'H' : "Invalid magic number in " + filePath;
        assert data.get(1) == 'A' : "Invalid magic number in " + filePath;
        assert data.get(2) == 'I' : "Invalid magic number in " + filePath;
        assert data.get(3) == 'R' : "Invalid magic number in " + filePath;

        IntBuffer intData = data.asIntBuffer();
        int featureFlags = intData.get(3); // should be 0x2:
        assert (featureFlags & 0x1) == 0x0 :
                "strands may contain different numbers of segments";
        assert (featureFlags & 0x2) == 0x2 : "no points defined";

        numStrands = intData.get(1);
        int numPoints = intData.get(2);

        ShortBuffer shortData = data.asShortBuffer();
        int numSegmentsPerStrand = 0xFFFF & shortData.get(8);
        numPointsPerStrand = numSegmentsPerStrand + 1;

        hairPoints = new ConstFloat3[numPoints];
        FloatBuffer floatData = data.asFloatBuffer();
        for (int pointI = 0; pointI < numPoints; ++pointI) {
            hairPoints[pointI] = new Float3(floatData, 32 + 3 * pointI);
        }
    }
}
