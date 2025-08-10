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
package com.github.stephengold.sportjolt.javaapp.image;

import com.github.stephengold.joltjni.BodyCreationSettings;
import com.github.stephengold.joltjni.BodyIdArray;
import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.BoxShapeSettings;
import com.github.stephengold.joltjni.CapsuleShapeSettings;
import com.github.stephengold.joltjni.CompoundShapeSettings;
import com.github.stephengold.joltjni.ConvexHullShapeSettings;
import com.github.stephengold.joltjni.CylinderShapeSettings;
import com.github.stephengold.joltjni.HeightFieldShape;
import com.github.stephengold.joltjni.HeightFieldShapeSettings;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.MassProperties;
import com.github.stephengold.joltjni.Mat44;
import com.github.stephengold.joltjni.MeshShapeSettings;
import com.github.stephengold.joltjni.MutableCompoundShapeSettings;
import com.github.stephengold.joltjni.OffsetCenterOfMassShapeSettings;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Plane;
import com.github.stephengold.joltjni.PlaneShape;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RotatedTranslatedShapeSettings;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.ShapeSettings;
import com.github.stephengold.joltjni.SphereShapeSettings;
import com.github.stephengold.joltjni.StaticCompoundShapeSettings;
import com.github.stephengold.joltjni.TaperedCapsuleShapeSettings;
import com.github.stephengold.joltjni.TaperedCylinderShapeSettings;
import com.github.stephengold.joltjni.TriangleShapeSettings;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.enumerate.EActivation;
import com.github.stephengold.joltjni.enumerate.EMotionType;
import com.github.stephengold.joltjni.enumerate.EOverrideMassProperties;
import com.github.stephengold.joltjni.enumerate.EShapeSubType;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstPlane;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Utils;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.Vertex;
import com.github.stephengold.sportjolt.importers.AssimpUtils;
import com.github.stephengold.sportjolt.input.InputProcessor;
import com.github.stephengold.sportjolt.input.RotateMode;
import com.github.stephengold.sportjolt.mesh.OctasphereMesh;
import com.github.stephengold.sportjolt.physics.BasePhysicsApp;
import com.github.stephengold.sportjolt.physics.ComGeometry;
import java.awt.image.BufferedImage;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.joml.Vector4f;
import org.lwjgl.glfw.GLFW;

/**
 * Generate images of various shapes.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class ShapeImages extends BasePhysicsApp {
    // *************************************************************************
    // fields

    /**
     * most recently displayed sample shape
     */
    private static ConstShape sampleShape;
    /**
     * currently selected type of shape
     */
    private static EShapeSubType selectedSubtype = EShapeSubType.Sphere;
    /**
     * height samples in the "mountains512" terrain model
     */
    private static FloatBuffer mountains512Heights;
    /**
     * triangles in the "viking_room" mesh
     */
    private static FloatBuffer vikingRoomTriangles;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the ShapeImages application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public ShapeImages() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the ShapeImages application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Logger.getLogger("").setLevel(Level.WARNING);
        ShapeImages application = new ShapeImages();
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
        int maxBodies = 4;
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
        configureCamera();
        configureInput();
        setBackgroundColor(new Vector4f(0.1f, 0.1f, 0.1f, 1f));
    }

    /**
     * Populate the PhysicsSystem. Invoked once during initialization.
     */
    @Override
    protected void populateSystem() {
        populateWith(selectedSubtype);
    }

    /**
     * Physics simulation is disabled.
     *
     * @param intervalSeconds the elapsed (real) time since the previous
     * invocation of {@code updatePhysics} (in seconds, &ge;0)
     */
    @Override
    protected void updatePhysics(float intervalSeconds) {
        super.updatePhysics(0f);
    }

    /**
     * Invoked before each frame is rendered, to update the text in the window's
     * title bar.
     */
    @Override
    protected void updateWindowTitle() {
        if (sampleShape == null) {
            return;
        }

        int ordinal = sampleShape.getSubType().ordinal();
        String className = sampleShape.getClass().getSimpleName();
        String typeString = sampleShape.getType().toString();
        boolean mustBeStatic = sampleShape.mustBeStatic();
        String predicates = mustBeStatic ? " (must be static)" : "";
        String title = String.format("[%d] %s : type=%s%s",
                ordinal, className, typeString, predicates);

        setWindowTitle(title);
    }
    // *************************************************************************
    // private methods

    /**
     * Append 3 box shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addBoxes(List<BodyCreationSettings> addSettings) {
        ShapeSettings ss = new BoxShapeSettings(0.5f, 0.8f, 1f);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new BoxShapeSettings(1.5f, 1.5f, 1.5f);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new BoxShapeSettings(0.1f, 0.1f, 0.6f);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(1., 1., 3.);
        bcs.setRotation(new Quat(0f, 0.6f, 0.8f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 capsule shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addCapsules(List<BodyCreationSettings> addSettings) {
        float halfHeight = 1f;
        float radius = 0.7f;
        ShapeSettings ss = new CapsuleShapeSettings(halfHeight, radius);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 0., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        halfHeight = 0.7f;
        radius = 1f;
        ss = new CapsuleShapeSettings(halfHeight, radius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append compound shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     * @param makeMutable {@code true} for a mutable shape or {@code false} for
     * a static shape
     */
    private static void addCompounds(
            List<BodyCreationSettings> addSettings, boolean makeMutable) {
        // Construct a cube-stick-ball shape:
        ShapeSettings boxSettings = new BoxShapeSettings(1f, 1f, 1f);
        float sphereRadius = 1f;
        ShapeSettings ballSettings = new SphereShapeSettings(sphereRadius);
        float halfHeight = 1f;
        float stickRadius = 0.3f;
        ShapeSettings stickSettings
                = new CylinderShapeSettings(halfHeight, stickRadius);
        CompoundShapeSettings compoundSettings;
        if (makeMutable) {
            compoundSettings = new MutableCompoundShapeSettings();
        } else {
            compoundSettings = new StaticCompoundShapeSettings();
        }
        compoundSettings.addShape(0f, -3.5f, 0f, ballSettings);
        compoundSettings.addShape(0f, -2f, 0f, stickSettings);
        compoundSettings.addShape(0f, 0f, 0f, boxSettings);
        compoundSettings.addShape(0f, 2f, 0f, stickSettings);
        compoundSettings.addShape(0f, 3.5f, 0f, ballSettings);
        ShapeRefC shapeRef = compoundSettings.create().get();

        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(-4., -1., -10.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
        /*
         * Construct a shape to approximate a torus (or donut),
         * open on the Z axis, using capsules arranged in a circle:
         */
        float majorRadius = 3f;
        float minorRadius = 0.5f;
        int numCapsules = 20;
        float arcLength = Constants.twoPi / numCapsules;
        float capsuleLength = majorRadius * arcLength;
        ShapeSettings capsuleSettings
                = new CapsuleShapeSettings(capsuleLength / 2f, minorRadius);
        if (makeMutable) {
            compoundSettings = new MutableCompoundShapeSettings();
        } else {
            compoundSettings = new StaticCompoundShapeSettings();
        }
        for (int childI = 0; childI < numCapsules; ++childI) {
            float theta = arcLength * childI;
            Vec3Arg offset = new Vec3(
                    majorRadius * Jolt.cos(theta),
                    majorRadius * Jolt.sin(theta),
                    0f);
            QuatArg rotation = Quat.sEulerAngles(0f, 0f, theta);
            compoundSettings.addShape(offset, rotation, capsuleSettings);
        }
        shapeRef = compoundSettings.create().get();

        bcs = new BodyCreationSettings();
        bcs.setPosition(-6., 0., -2.);
        bcs.setRotation(new Quat(0f, 0.6f, 0.8f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 convex-hull shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addConvexHulls(List<BodyCreationSettings> addSettings) {
        OctasphereMesh mesh = new OctasphereMesh(0, false);
        int numVertices = 3 * mesh.countTriangles();
        FloatBuffer positionFloats = mesh.getPositionsData();
        float radius = 1.8f;
        for (int i = 0; i < positionFloats.capacity(); ++i) {
            float f = positionFloats.get(i);
            positionFloats.put(i, radius * f);
        }
        ShapeSettings ss
                = new ConvexHullShapeSettings(numVertices, positionFloats);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 0., 1.);
        bcs.setRotation(new Quat(0f, 0.6f, 0f, -0.8f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        float noseZ = 1.4f;           // offset from chassis center
        float spoilerY = 0.5f;        // offset from chassis center
        float tailZ = -0.7f;          // offset from chassis center
        float undercarriageY = -0.1f; // offset from chassis center
        float halfWidth = 0.4f;
        Collection<Vec3Arg> cornerLocations = new ArrayList<>(6);
        cornerLocations.add(new Vec3(+halfWidth, undercarriageY, noseZ));
        cornerLocations.add(new Vec3(-halfWidth, undercarriageY, noseZ));
        cornerLocations.add(new Vec3(+halfWidth, undercarriageY, tailZ));
        cornerLocations.add(new Vec3(-halfWidth, undercarriageY, tailZ));
        cornerLocations.add(new Vec3(+halfWidth, spoilerY, tailZ));
        cornerLocations.add(new Vec3(-halfWidth, spoilerY, tailZ));
        ss = new ConvexHullShapeSettings(cornerLocations);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(1., 1., -1.);
        bcs.setRotation(new Quat(-0.5f, 0.5f, 0.5f, 0.5f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 cylinder shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addCylinders(List<BodyCreationSettings> addSettings) {
        float halfHeight = 1f;
        float radius = 0.7f;
        ShapeSettings ss = new CylinderShapeSettings(halfHeight, radius);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 0., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        halfHeight = 0.7f;
        radius = 1f;
        ss = new CylinderShapeSettings(halfHeight, radius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append a height-field shape to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addHeightField(List<BodyCreationSettings> addSettings) {
        FloatBuffer heightBuffer = loadMountains512Heights();
        int numFloats = heightBuffer.capacity();

        Vec3Arg offset = new Vec3(0f, 0f, 0f);
        Vec3Arg scale = new Vec3(0.006f, 0.006f, 0.006f);
        int sampleCount = 512;
        assert numFloats == sampleCount * sampleCount;
        ShapeSettings ss = new HeightFieldShapeSettings(
                heightBuffer, offset, scale, sampleCount);

        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 1.5, 0.4);
        bcs.setRotation(Quat.sEulerAngles(0.1f, 0f, -0.1f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append a mesh shape to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addMesh(List<BodyCreationSettings> addSettings) {
        FloatBuffer floatBuffer = loadVikingRoomTriangles();
        ShapeSettings ss = new MeshShapeSettings(floatBuffer);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(3., 1.3, 3.5);
        bcs.setRotation(new Quat(0.5f, 0.5f, 0.5f, -0.5f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append a plane shape to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addPlane(List<BodyCreationSettings> addSettings) {
        Vec3Arg normal = new Vec3(0f, 5f, 1f).normalized();
        ConstPlane plane = new Plane(normal, 0f);
        PlaneShape shape = new PlaneShape(plane);
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(shape);
        addSettings.add(bcs);
    }

    /**
     * Append offset center-of-mass shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addOcoms(List<BodyCreationSettings> addSettings) {
        float sphereRadius = 1f;
        ShapeSettings ss = new SphereShapeSettings(sphereRadius);
        Vec3Arg offset = new Vec3(0f, -0.9f, 0f);
        ss = new OffsetCenterOfMassShapeSettings(offset, ss);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(-2., 1., 2.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new BoxShapeSettings(1.5f, 1.5f, 1.5f);
        offset = new Vec3(-1.5f, -1.5f, 1.5f);
        ss = new OffsetCenterOfMassShapeSettings(offset, ss);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -2.5);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 4 rotated-translated shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addRts(List<BodyCreationSettings> addSettings) {
        float halfHeight = 1f;
        float radius = 0.7f;
        ShapeSettings ss = new CylinderShapeSettings(halfHeight, radius);
        Vec3Arg offset = new Vec3(0f, -1f, 0f);
        QuatArg rotation = new Quat();
        ss = new RotatedTranslatedShapeSettings(offset, rotation, ss);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new CylinderShapeSettings(halfHeight, radius);
        offset = new Vec3(0f, 1f, 0f);
        rotation = new Quat();
        ss = new RotatedTranslatedShapeSettings(offset, rotation, ss);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-2.2, -1., 2.2);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new CylinderShapeSettings(halfHeight, radius);
        offset = new Vec3();
        rotation = new Quat(0.5f, 0.5f, 0.5f, 0.5f);
        ss = new RotatedTranslatedShapeSettings(offset, rotation, ss);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(0., 2., 0.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        ss = new CylinderShapeSettings(halfHeight, radius);
        offset = new Vec3();
        rotation = new Quat(0.5f, 0.5f, 0.5f, -0.5f);
        ss = new RotatedTranslatedShapeSettings(offset, rotation, ss);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(1., -0.5, 0.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 3 sphere shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private void addSpheres(List<BodyCreationSettings> addSettings) {
        float sphereRadius = 1f;
        ShapeSettings ss = new SphereShapeSettings(sphereRadius);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        sphereRadius = 1.5f;
        ss = new SphereShapeSettings(sphereRadius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        sphereRadius = 0.4f;
        ss = new SphereShapeSettings(sphereRadius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(1., 1., 3.);
        bcs.setRotation(new Quat(0f, 0.6f, 0.8f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 tapered capsule shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addTaperedCapsules(
            List<BodyCreationSettings> addSettings) {
        float bottomRadius = 1f;
        float halfHeight = 0.3f;
        float topRadius = 0.8f;
        ShapeSettings ss = new TaperedCapsuleShapeSettings(
                halfHeight, topRadius, bottomRadius);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 0., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        bottomRadius = 0.7f;
        halfHeight = 2f;
        topRadius = 0.1f;
        ss = new TaperedCapsuleShapeSettings(
                halfHeight, topRadius, bottomRadius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 tapered cylinder shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addTaperedCylinders(
            List<BodyCreationSettings> addSettings) {
        float bottomRadius = 1f;
        float halfHeight = 0.3f;
        float topRadius = 0.8f;
        ShapeSettings ss = new TaperedCylinderShapeSettings(
                halfHeight, topRadius, bottomRadius);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setPosition(0., 0., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        bottomRadius = 0.7f;
        halfHeight = 2f;
        topRadius = 0.1f;
        ss = new TaperedCylinderShapeSettings(
                halfHeight, topRadius, bottomRadius);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setPosition(-1., 1.5, -3.);
        bcs.setRotation(new Quat(0.6f, -0.8f, 0f, 0f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Append 2 triangle shapes to the specified list.
     *
     * @param addSettings the list to modify (not null, added to)
     */
    private static void addTriangles(List<BodyCreationSettings> addSettings) {
        Vec3Arg v1 = new Vec3(-1f, -0.8f, 2f);
        Vec3Arg v2 = new Vec3(2f, -0.8f, -1f);
        Vec3Arg v3 = new Vec3(-1f, 1.6f, -1f);
        ShapeSettings ss = new TriangleShapeSettings(v1, v2, v3);
        ShapeRefC shapeRef = ss.create().get();
        BodyCreationSettings bcs = new BodyCreationSettings();
        bcs.setOverrideMassProperties(
                EOverrideMassProperties.MassAndInertiaProvided);
        bcs.setPosition(1., 1.2, 2.);
        bcs.setRotation(new Quat(0.5f, -0.5f, 0.5f, -0.5f));
        bcs.setShape(shapeRef);
        addSettings.add(bcs);

        v1 = new Vec3(0f, -0.5f, 0f);
        v2 = new Vec3(1f, 0.5f, 0f);
        v3 = new Vec3(-1f, 0.5f, 0f);
        ss = new TriangleShapeSettings(v1, v2, v3);
        shapeRef = ss.create().get();
        bcs = new BodyCreationSettings();
        bcs.setOverrideMassProperties(
                EOverrideMassProperties.MassAndInertiaProvided);
        bcs.setPosition(-2., 1., 1.);
        bcs.setShape(shapeRef);
        addSettings.add(bcs);
    }

    /**
     * Advance the selected shape subtype by the specified amount.
     *
     * @param amount the number of values to advance (not zero, may be negative)
     */
    private void advanceSelectedSubtype(int amount) {
        Validate.nonZero(amount, "amount");

        cleanUp();

        EShapeSubType[] values = EShapeSubType.values();
        int numSubtypes = values.length;
        int ordinal = selectedSubtype.ordinal();

        boolean success;
        while (true) {
            ordinal = Utils.modulo(ordinal + amount, numSubtypes);
            EShapeSubType subtype = values[ordinal];
            success = populateWith(subtype);
            if (success) {
                selectedSubtype = subtype;
                return;
            }
            amount = (amount > 0) ? +1 : -1;
        }
    }

    /**
     * Configure the camera and CIP during initialization.
     */
    private static void configureCamera() {
        getCameraInputProcessor()
                .setMoveSpeed(3f)
                .setRotationMode(RotateMode.DragLMB);

        cam.setLocation(4.6f, 2f, 5.2f)
                .setAzimuth(-2.35f)
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
                    case GLFW.GLFW_KEY_ENTER:
                    case GLFW.GLFW_KEY_KP_6:
                    case GLFW.GLFW_KEY_N:
                    case GLFW.GLFW_KEY_SPACE:
                        if (isPressed) {
                            advanceSelectedSubtype(+1);
                        }
                        return;

                    case GLFW.GLFW_KEY_BACKSPACE:
                    case GLFW.GLFW_KEY_KP_4:
                    case GLFW.GLFW_KEY_P:
                        if (isPressed) {
                            advanceSelectedSubtype(-1);
                        }
                        return;

                    default:
                }
                super.onKeyboard(keyId, isPressed);
            }
        });
    }

    /**
     * Generate body-creation settings for representative shapes of the
     * specified subtype.
     *
     * @param subtype the type of shape to create (not null)
     * @return a new list of new objects
     */
    private List<BodyCreationSettings> generateSettingsFor(
            EShapeSubType subtype) {
        List<BodyCreationSettings> result = new ArrayList<>(3);
        switch (subtype) {
            case Box:
                addBoxes(result);
                break;
            case Capsule:
                addCapsules(result);
                break;
            case ConvexHull:
                addConvexHulls(result);
                break;
            case Cylinder:
                addCylinders(result);
                break;
            case HeightField:
                addHeightField(result);
                break;
            case Mesh:
                addMesh(result);
                break;
            case MutableCompound:
                addCompounds(result, true);
                break;
            case OffsetCenterOfMass:
                addOcoms(result);
                break;
            case Plane:
                addPlane(result);
                break;
            case RotatedTranslated:
                addRts(result);
                break;
            case Sphere:
                addSpheres(result);
                break;
            case TaperedCapsule:
                addTaperedCapsules(result);
                break;
            case TaperedCylinder:
                addTaperedCylinders(result);
                break;
            case Triangle:
                addTriangles(result);
                break;
            default:
        }

        return result;
    }

    /**
     * Load height samples from the "mountains512" terrain model and cut a hole
     * in the samples.
     *
     * @return a 512x512 array of height samples
     */
    private static FloatBuffer loadMountains512Heights() {
        if (mountains512Heights == null) {
            // Generate an array of heights from a PNG image on the classpath:
            String resourceName = "/Textures/Terrain/splat/mountains512.png";
            BufferedImage image = Utils.loadResourceAsImage(resourceName);
            float maxHeight = 51f;
            mountains512Heights = Utils.toHeightBuffer(image, maxHeight);

            // Cut a 15x30 rectangular hole in the samples:
            int sampleCount = 512;
            for (int i = 480; i < 495; ++i) {
                for (int j = 470; j < 500; ++j) {
                    int floatIndex = i + sampleCount * j;
                    mountains512Heights.put(floatIndex, Float.MAX_VALUE);
                }
            }
        }

        return mountains512Heights;
    }

    /**
     * Load triangles from the "viking_room" mesh.
     *
     * @return a direct buffer with capacity a multiple of 9
     */
    private static FloatBuffer loadVikingRoomTriangles() {
        if (vikingRoomTriangles == null) {
            // Load the model using Assimp:
            String modelName = "/Models/viking_room/viking_room.obj";
            int postFlags = 0x0;
            List<Integer> indices = null;
            List<Vertex> vertices = new ArrayList<>();
            AssimpUtils.extractTriangles(
                    modelName, postFlags, indices, vertices);

            int numVertices = vertices.size();
            assert numVertices % 3 == 0;
            vikingRoomTriangles = Jolt.newDirectFloatBuffer(3 * numVertices);
            for (int i = 0; i < numVertices; ++i) {
                Vertex vertex = vertices.get(i);
                vertex.writePositionTo(vikingRoomTriangles);
            }
        }
        return vikingRoomTriangles;
    }

    /**
     * Populate the physics space with representative shapes of the specified
     * subtype.
     *
     * @param subtype the type of shape to create (not null)
     * @return {@code true} if successful, otherwise {@code false}
     */
    private boolean populateWith(EShapeSubType subtype) {
        List<BodyCreationSettings> settings = generateSettingsFor(subtype);
        int numBodies = settings.size();
        if (numBodies == 0) {
            return false;
        }

        // Create the bodies and store them in an array:
        BodyIdArray bodyIds = new BodyIdArray(numBodies);
        BodyInterface bi = physicsSystem.getBodyInterface();
        for (int bodyIndex = 0; bodyIndex < numBodies; ++bodyIndex) {
            BodyCreationSettings bcs = settings.get(bodyIndex);
            ConstShape shape = bcs.getShape();
            sampleShape = shape;
            /*
             * Some shapes cannot calculate mass and inertia automatically,
             * so provide arbitrary values:
             */
            MassProperties mpo = bcs.getMassPropertiesOverride();
            mpo.setInertia(Mat44.sIdentity())
                    .setMass(1f);

            boolean makeStatic = shape.mustBeStatic();
            if (makeStatic) {
                bcs.setMotionType(EMotionType.Static);
                bcs.setObjectLayer(objLayerNonMoving);
            } else {
                bcs.setAllowSleeping(false);
                bcs.setGravityFactor(0f);
                bcs.setMotionType(EMotionType.Dynamic);
                bcs.setObjectLayer(objLayerMoving);
            }

            ConstBody body = bi.createBody(bcs);
            int bodyId = body.getId();
            bodyIds.set(bodyIndex, bodyId);

            float axisLength = 1f;
            visualizeAxes(body, axisLength);
            if (!makeStatic) {
                new ComGeometry(body).setDepthTest(false);
            }

            if (shape instanceof HeightFieldShape) {
                visualizeShape(body).setProgram("Phong/Distant/Monochrome");
            } else {
                visualizeShape(body);
            }
        }

        // Add the bodies to the physics space:
        long handle = bi.addBodiesPrepare(bodyIds, numBodies);
        bi.addBodiesFinalize(bodyIds, numBodies, handle, EActivation.Activate);

        return true;
    }
}
