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
package com.github.stephengold.sportjolt.physics;

import com.github.stephengold.joltjni.BodyLockRead;
import com.github.stephengold.joltjni.BroadPhaseLayerInterface;
import com.github.stephengold.joltjni.BroadPhaseLayerInterfaceTable;
import com.github.stephengold.joltjni.JobSystem;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.JoltPhysicsObject;
import com.github.stephengold.joltjni.ObjectLayerPairFilterTable;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilter;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilterTable;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorMalloc;
import com.github.stephengold.joltjni.VehicleConstraint;
import com.github.stephengold.joltjni.enumerate.EPhysicsUpdateError;
import com.github.stephengold.joltjni.enumerate.EShapeSubType;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstBodyLockInterface;
import com.github.stephengold.joltjni.readonly.ConstCharacter;
import com.github.stephengold.joltjni.readonly.ConstCharacterVirtual;
import com.github.stephengold.joltjni.readonly.ConstJoltPhysicsObject;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Filter;
import com.github.stephengold.sportjolt.FlipAxes;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.NormalsOption;
import com.github.stephengold.sportjolt.TextureKey;
import com.github.stephengold.sportjolt.UvsOption;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.WrapFunction;
import electrostatic4j.snaploader.LibraryInfo;
import electrostatic4j.snaploader.LoadingCriterion;
import electrostatic4j.snaploader.NativeBinaryLoader;
import electrostatic4j.snaploader.filesystem.DirectoryPath;
import electrostatic4j.snaploader.platform.NativeDynamicLibrary;
import electrostatic4j.snaploader.platform.util.PlatformPredicate;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.WeakHashMap;
import org.joml.Vector4f;

/**
 * An application to visualize 3-D physics.
 */
public abstract class BasePhysicsApp extends BaseApplication {
    // *************************************************************************
    // constants

    /**
     * customary number of object layers
     */
    final public static int numObjLayers = 2;
    /**
     * number of worker threads to use
     */
    final private static int numWorkerThreads
            = Runtime.getRuntime().availableProcessors();
    /**
     * customary object layer for moving objects
     */
    final public static int objLayerMoving = 0;
    /**
     * customary object layer for non-moving objects
     */
    final public static int objLayerNonMoving = 1;
    /**
     * expected version string of the Jolt-JNI native library
     */
    final private static String expectedVersion = "3.3.0";
    // *************************************************************************
    // fields

    /**
     * callbacks invoked before each simulation step
     */
    final private static Collection<PhysicsTickListener> tickListeners
            = new ArrayList<>(5);
    /**
     * total time simulated (in simulated seconds)
     */
    private static double totalSimulatedTime;
    /**
     * simulation lag (for {@code maxSubSteps>0}, in seconds, &ge;0)
     */
    private float physicsLag;
    /**
     * time step (in seconds, &gt;0)
     */
    protected float timePerStep = 1f / 60f;
    /**
     * how many times render() has been invoked
     */
    private int renderCount;
    /**
     * schedule simulation jobs
     */
    private JobSystem jobSystem;
    /**
     * timestamp of the previous render() (for {@code renderCount>0}, in
     * nanoseconds)
     */
    private long lastPhysicsUpdate;
    /**
     * accumulated wall-clock time spent in physics simulation, including step
     * listeners (in nanoseconds)
     */
    private static long totalPhysicsNanos;
    /**
     * map shape summaries to auto-generated meshes, for reuse
     */
    final private static Map<ShapeSummary, Mesh> meshCache
            = new WeakHashMap<>(200);
    /**
     * system for physics simulation
     */
    protected PhysicsSystem physicsSystem;
    /**
     * allocate temporary memory for physics simulation
     */
    private TempAllocator tempAllocator;
    // *************************************************************************
    // constructors

    /**
     * Explicit no-arg constructor to avoid javadoc warnings from JDK 18+.
     */
    protected BasePhysicsApp() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Load and initialize the Jolt-JNI native library. Intended for internal
     * use of the Sport-Jolt library.
     *
     * @param traceAllocations {@code true} to trace C++ heap allocations in
     * Debug native libraries, {@code false} to disable tracing (default=false)
     * @param startCleaner {@code true} to start a cleaner that automatically
     * reclaims Jolt-JNI native objects, {@code false} to leave the app
     * responsible for reclaiming them
     */
    public static void initializeJoltJni(
            boolean traceAllocations, boolean startCleaner) {
        PlatformPredicate linuxWithFma = new PlatformPredicate(
                PlatformPredicate.LINUX_X86_64,
                "avx", "avx2", "bmi1", "f16c", "fma", "sse4_1", "sse4_2");
        PlatformPredicate windowsWithAvx2 = new PlatformPredicate(
                PlatformPredicate.WIN_X86_64,
                "avx", "avx2", "sse4_1", "sse4_2");

        LibraryInfo info
                = new LibraryInfo(null, "joltjni", DirectoryPath.USER_DIR);
        NativeBinaryLoader loader = new NativeBinaryLoader(info);

        NativeDynamicLibrary[] libraries = {
            new NativeDynamicLibrary("linux/aarch64/com/github/stephengold",
                PlatformPredicate.LINUX_ARM_64),
            new NativeDynamicLibrary("linux/armhf/com/github/stephengold",
                PlatformPredicate.LINUX_ARM_32),
            new NativeDynamicLibrary(
                "linux/x86-64-fma/com/github/stephengold",
                linuxWithFma), // This should precede vanilla LINUX_X86_64.

            new NativeDynamicLibrary("linux/x86-64/com/github/stephengold",
                PlatformPredicate.LINUX_X86_64),
            new NativeDynamicLibrary("osx/aarch64/com/github/stephengold",
                PlatformPredicate.MACOS_ARM_64),
            new NativeDynamicLibrary("osx/x86-64/com/github/stephengold",
                PlatformPredicate.MACOS_X86_64),
            new NativeDynamicLibrary(
                "windows/x86-64-avx2/com/github/stephengold",
                windowsWithAvx2), // This should precede vanilla WIN_X86_64.

            new NativeDynamicLibrary("windows/x86-64/com/github/stephengold",
            PlatformPredicate.WIN_X86_64)
        };
        loader.registerNativeLibraries(libraries)
                .initPlatformLibrary()
                .setRetryWithCleanExtraction(true);

        // Load a Jolt-JNI native library for this platform:
        try {
            loader.loadLibrary(LoadingCriterion.CLEAN_EXTRACTION);
        } catch (Exception exception) {
            throw new IllegalStateException(
                    "Failed to load a Jolt-JNI native library!", exception);
        }

        printLibraryInfo(System.out);

        String jjVersion = Jolt.versionString();
        if (!jjVersion.equals(expectedVersion)) {
            System.err.println("Expected a v" + expectedVersion
                    + " native library but loaded v" + jjVersion + "!");
            System.err.flush();
        }

        if (Jolt.buildType().equals("Debug")) {
            Jolt.setTraceAllocations(traceAllocations); // default=false
        }
        if (startCleaner) {
            JoltPhysicsObject.startCleaner(); // to reclaim native memory
        }

        Jolt.registerDefaultAllocator();
        Jolt.installDefaultAssertCallback();
        Jolt.installDefaultTraceCallback();

        boolean success = Jolt.newFactory();
        assert success;
        Jolt.registerTypes();
    }

    /**
     * Return a Mesh to visualize the summarized shape.
     *
     * @param shape the shape to visualize (not null, unaffected)
     * @param summary a summary of the shape (not null)
     * @return an immutable Mesh (not null)
     */
    public static Mesh meshForShape(ConstShape shape, ShapeSummary summary) {
        Mesh result;

        if (meshCache.containsKey(summary)) {
            result = meshCache.get(summary);

        } else {
            //System.out.printf("Generate mesh for %s va=%s strategy=%s%n",
            //        shape.getSubType(), Long.toHexString(shape.targetVa()),
            //        summary.meshingStrategy());
            MeshingStrategy strategy = summary.meshingStrategy();
            result = strategy.applyTo(shape);
            result.makeImmutable();
            meshCache.put(summary, result);
        }

        return result;
    }

    /**
     * Visualize the local axes of the specified physics object.
     *
     * @param jpo the object to visualize (or null for world axes)
     * @param axisLength how much of each axis to visualize (in world units,
     * &ge;0)
     * @return an array of new, visible geometries
     */
    public static Geometry[] visualizeAxes(
            ConstJoltPhysicsObject jpo, float axisLength) {
        Validate.nonNegative(axisLength, "axis length");

        int numAxes = 3;
        Geometry[] result = new Geometry[numAxes];

        for (int ai = 0; ai < numAxes; ++ai) {
            result[ai] = new LocalAxisGeometry(jpo, ai, axisLength)
                    .setDepthTest(false);
        }

        return result;
    }

    /**
     * Visualize the shape of the specified body, with locking.
     *
     * @param bodyId the ID of the body to visualize
     * @return a new, visible Geometry
     */
    public Geometry visualizeBodyShape(int bodyId) {
        ConstBodyLockInterface bli = physicsSystem.getBodyLockInterface();
        BodyLockRead lock = new BodyLockRead(bli, bodyId);
        assert lock.succeeded();

        ConstBody body = lock.getBody();
        Geometry geometry = visualizeShape(body);

        lock.releaseLock();
        return geometry;
    }

    /**
     * Visualize all Cosserat rods in the specified soft body, with locking.
     *
     * @param bodyId the ID of the soft body to visualize
     * @return a new, visible Geometry
     */
    public Geometry visualizeRods(int bodyId) {
        ConstBodyLockInterface bli = physicsSystem.getBodyLockInterface();
        BodyLockRead lock = new BodyLockRead(bli, bodyId);
        assert lock.succeeded();

        ConstBody body = lock.getBody();
        Geometry geometry = new RodsGeometry(body);

        lock.releaseLock();
        return geometry;
    }

    /**
     * Visualize the collision shape of the specified physics object.
     *
     * @param jpo the physics object to visualize (not null)
     * @return a new, visible Geometry
     */
    public static Geometry visualizeShape(ConstJoltPhysicsObject jpo) {
        float uvScale = 1f;
        Geometry result = visualizeShape(jpo, uvScale);

        return result;
    }

    /**
     * Visualize the shape of the specified physics object.
     *
     * @param jpo the physics object to visualize (not null)
     * @param uvScale the UV scale factor to use (default=1)
     * @return a new, visible Geometry
     */
    public static Geometry visualizeShape(
            ConstJoltPhysicsObject jpo, float uvScale) {
        ConstShape shape = getShape(jpo);

        MeshingStrategy meshingStrategy;
        String programName;
        TextureKey textureKey;

        EShapeSubType subType = shape.getSubType();
        if (subType == EShapeSubType.Plane
                || subType == EShapeSubType.HeightField) {
            int positionStrategy = 0;
            meshingStrategy = new MeshingStrategy(
                    positionStrategy, NormalsOption.Facet, UvsOption.Linear,
                    new Vector4f(uvScale, 0f, 0f, 0f),
                    new Vector4f(0f, 0f, uvScale, 0f)
            );
            programName = "Phong/Distant/Texture";
            boolean mipmaps = true;
            float maxAniso = 16f;
            textureKey = new TextureKey("procedural:///checkerboard?size=128",
                    Filter.Linear, Filter.NearestMipmapLinear,
                    WrapFunction.Repeat, WrapFunction.Repeat, mipmaps,
                    FlipAxes.noFlip, maxAniso);

        } else if (subType == EShapeSubType.Sphere) {
            int positionStrategy = -3;
            meshingStrategy = new MeshingStrategy(
                    positionStrategy, NormalsOption.Sphere, UvsOption.Spherical,
                    new Vector4f(uvScale, 0f, 0f, 0f),
                    new Vector4f(0f, uvScale, 0f, 0f)
            );
            programName = "Phong/Distant/Texture";
            textureKey = new TextureKey(
                    "procedural:///checkerboard?size=2&color0=999999ff",
                    Filter.Nearest, Filter.Nearest);

        } else { // shape isn't a heightfield, plane, or sphere:
            programName = "Phong/Distant/Monochrome";
            textureKey = null;

            if (subType == EShapeSubType.Capsule
                    || subType == EShapeSubType.TaperedCapsule) {
                meshingStrategy = new MeshingStrategy("low/Smooth");
            } else {
                meshingStrategy = new MeshingStrategy("low/Facet");
            }
        }

        Geometry geometry;
        if (jpo instanceof ConstBody) {
            ConstBody body = (ConstBody) jpo;
            if (body.isSoftBody()) {
                geometry = new FacesGeometry(body);
            } else {
                geometry = new RigidBodyShapeGeometry(body, meshingStrategy);
            }

        } else if (jpo instanceof ConstCharacter) {
            ConstCharacter character = (ConstCharacter) jpo;
            geometry = new CharacterShapeGeometry(character, meshingStrategy);

        } else if (jpo instanceof ConstCharacterVirtual) {
            ConstCharacterVirtual character = (ConstCharacterVirtual) jpo;
            geometry = new CharacterVirtualShapeGeometry(
                    character, meshingStrategy);

        } else if (jpo instanceof VehicleConstraint) {
            VehicleConstraint constraint = (VehicleConstraint) jpo;
            ConstBody body = constraint.getVehicleBody();
            geometry = new RigidBodyShapeGeometry(body, meshingStrategy);

        } else {
            throw new IllegalArgumentException(jpo.toString());
        }

        geometry.setProgram(programName);
        geometry.setSpecularColor(Constants.GRAY);
        if (textureKey != null) {
            geometry.setTexture(textureKey);
        }

        return geometry;
    }

    /**
     * Visualize the wheels of the specified vehicle.
     *
     * @param vehicle the vehicle to visualize (not null)
     * @return an array of new, visible geometries
     */
    public static Geometry[] visualizeWheels(VehicleConstraint vehicle) {
        int numWheels = vehicle.countWheels();
        Geometry[] result = new Geometry[numWheels];
        for (int wheelIndex = 0; wheelIndex < numWheels; ++wheelIndex) {
            result[wheelIndex] = new WheelGeometry(vehicle, wheelIndex);
        }

        return result;
    }
    // *************************************************************************
    // protected methods

    /**
     * Add the specified physics-tick listener.
     *
     * @param listener the listener to add (not null, alias created)
     */
    protected void addTickListener(PhysicsTickListener listener) {
        tickListeners.add(listener);
    }

    /**
     * Create the PhysicsSystem during initialization. Meant to be overridden.
     *
     * @return a new object
     */
    protected PhysicsSystem createSystem() {
        int maxBodies = 100;
        int numBpLayers = 2;
        PhysicsSystem result = createSystem(maxBodies, numBpLayers);

        return result;
    }

    /**
     * Create a generic {@code PhysicsSystem} with the specified parameters.
     * Heuristics are used to configure reasonable limits for mutexes, pairs,
     * and contacts.
     *
     * @param maxBodies the maximum number of bodies (&ge;1)
     * @param numBpLayers the number of broadphase layers (1 or 2)
     * @return a new object
     */
    protected PhysicsSystem createSystem(int maxBodies, int numBpLayers) {
        Validate.positive(maxBodies, "maximum number of bodies");
        Validate.inRange(numBpLayers, "number of BP layers", 1, 2);
        /*
         * The number of object layers in Sport-Jolt is fixed at 2:
         * one for moving objects and one for non-moving ones.
         *
         * Configure an object-layer pair filter to ignore collisions
         * between non-moving objects:
         */
        ObjectLayerPairFilterTable ovoFilter
                = new ObjectLayerPairFilterTable(numObjLayers);
        ovoFilter.enableCollision(objLayerMoving, objLayerMoving);
        ovoFilter.enableCollision(objLayerMoving, objLayerNonMoving);

        BroadPhaseLayerInterface layerMap = createLayerMap(numBpLayers);
        /*
         * Pre-compute the rules for colliding object layers
         * with broadphase layers:
         */
        ObjectVsBroadPhaseLayerFilter ovbFilter
                = new ObjectVsBroadPhaseLayerFilterTable(
                        layerMap, numBpLayers, ovoFilter, numObjLayers);

        int numBodyMutexes = 0; // 0 means "use the default value"
        /*
         * Current heuristics:
         *
         * maxBodies=2 -> maxBodyPairs=3, maxContacts=56
         * maxBodies=3 -> maxBodyPairs=3, maxContacts=63
         * maxBodies=10 -> maxBodyPairs=45, maxContacts=112
         * maxBodies=100 -> maxBodyPairs=1200, maxContacts=742
         * maxBodies=1000 -> maxBodyPairs=3900, maxContacts=7042
         */
        long maxBodiesLong = maxBodies;
        long possiblePairs = maxBodiesLong * (maxBodiesLong - 1) / 2;
        int maxBodyPairs = (int) Math.min(possiblePairs, 3 * maxBodies + 900);
        maxBodyPairs = Math.max(3, maxBodyPairs);

        int maxContacts = 7 * (maxBodies + 6);
        PhysicsSystem result = new PhysicsSystem();
        result.init(maxBodies, numBodyMutexes, maxBodyPairs, maxContacts,
                layerMap, ovbFilter, ovoFilter);

        return result;
    }

    /**
     * Access the physics system.
     *
     * @return the pre-existing instance
     */
    protected PhysicsSystem getPhysicsSystem() {
        return physicsSystem;
    }

    /**
     * Access the temporary memory allocator.
     *
     * @return the pre-existing instance
     */
    protected TempAllocator getTempAllocator() {
        return tempAllocator;
    }

    /**
     * Populate the PhysicsSystem. Invoked once during initialization.
     */
    abstract protected void populateSystem();

    /**
     * Advance the physics simulation by the specified interval. Invoked during
     * each update. Meant to be overridden.
     *
     * @param intervalSeconds the elapsed (real) time since the previous
     * invocation of {@code updatePhysics} (in seconds, &ge;0)
     */
    protected void updatePhysics(float intervalSeconds) {
        assert physicsLag >= 0f : physicsLag;
        float timeSinceStep = physicsLag + intervalSeconds;
        int numSubSteps = (int) Math.floor(timeSinceStep / timePerStep);
        assert numSubSteps >= 0 : numSubSteps;
        this.physicsLag = timeSinceStep - numSubSteps * timePerStep;
        assert physicsLag >= 0f : physicsLag;

        if (numSubSteps > 4) {
            numSubSteps = 4;
        }

        for (int i = 0; i < numSubSteps; ++i) {
            // Notify any step listeners:
            for (PhysicsTickListener listeners : tickListeners) {
                listeners.prePhysicsTick(physicsSystem, timePerStep);
            }

            // Single-step the physics system:
            int collisionSteps = 1;
            int errors = physicsSystem.update(
                    timePerStep, collisionSteps, tempAllocator, jobSystem);
            assert errors == EPhysicsUpdateError.None : errors;
            totalSimulatedTime += collisionSteps * timePerStep;

            // Notify any step listeners:
            for (PhysicsTickListener listeners : tickListeners) {
                listeners.physicsTick(physicsSystem, timePerStep);
            }
        }
    }
    // *************************************************************************
    // BaseApplication methods

    /**
     * Callback invoked after the main update loop terminates.
     */
    @Override
    protected void cleanUp() {
        physicsSystem.removeAllConstraints();
        physicsSystem.destroyAllBodies();

        Collection<Geometry> geometries = listVisible();
        hideAll(geometries);

        // Discard all meshes auto-generated for collision shapes:
        for (Mesh mesh : meshCache.values()) {
            mesh.cleanUp();
        }
        meshCache.clear();
    }

    /**
     * Callback invoked before the main update loop begins. Meant to be
     * overridden.
     */
    @Override
    protected void initialize() {
        this.tempAllocator = new TempAllocatorMalloc();
        this.jobSystem = new JobSystemThreadPool(Jolt.cMaxPhysicsJobs,
                Jolt.cMaxPhysicsBarriers, numWorkerThreads);
        this.physicsSystem = createSystem();
        populateSystem();
        physicsSystem.optimizeBroadPhase();
    }

    /**
     * Callback invoked during each iteration of the main update loop. Meant to
     * be overridden.
     */
    @Override
    protected void render() {
        ++renderCount;

        // Advance the physics, but not during the first render().
        long nanoTime = System.nanoTime();
        if (renderCount > 1) {
            long nanoseconds = nanoTime - lastPhysicsUpdate;
            float seconds = 1e-9f * nanoseconds;
            updatePhysics(seconds);

            long nanoTimeAfter = System.nanoTime();
            long updateNanos = nanoTimeAfter - nanoTime;
            totalPhysicsNanos += updateNanos;
        }
        this.lastPhysicsUpdate = nanoTime;

        cleanUpGeometries();
        super.render();
    }

    /**
     * Return the accumulated wall-clock time spent in physics simulation,
     * including step listeners.
     *
     * @return the total time (in nanoseconds)
     */
    protected static long totalPhysicsNanos() {
        return totalPhysicsNanos;
    }

    /**
     * Return the total time simulated.
     *
     * @return the total time (in simulated seconds)
     */
    protected static double totalSimulatedTime() {
        return totalSimulatedTime;
    }
    // *************************************************************************
    // private methods

    /**
     * Hide any geometries associated with physics objects that are no longer in
     * the PhysicsSystem.
     */
    private void cleanUpGeometries() {
        Collection<Geometry> geometriesToHide
                = new ArrayList<>(); // TODO garbage
        //System.out.println();
        for (Geometry geometry : listVisible()) {
            //System.out.println(
            //       "geometry type=" + geometry.getClass().getSimpleName());
            //System.out.println("  " + geometry.toString());
            if (geometry.wasRemovedFrom(physicsSystem)) {
                geometriesToHide.add(geometry);
            }
        }

        hideAll(geometriesToHide);
    }

    /**
     * Generate a mapping from object layers to broadphase layers.
     *
     * @param numBpLayers the desired number of broadphase layers (1 or 2)
     * @return a new object
     */
    private static BroadPhaseLayerInterface createLayerMap(int numBpLayers) {
        BroadPhaseLayerInterfaceTable result
                = new BroadPhaseLayerInterfaceTable(numObjLayers, numBpLayers);
        if (numBpLayers == 2) {
            /*
             * Identify non-moving objects sooner
             * by mapping them to a separate broadphase layer:
             */
            int bpLayerMoving = 0;
            int bpLayerNonMoving = 1;
            result.mapObjectToBroadPhaseLayer(objLayerMoving, bpLayerMoving);
            result.mapObjectToBroadPhaseLayer(
                    objLayerNonMoving, bpLayerNonMoving);

        } else {
            assert numBpLayers == 1 : "numBpLayers = " + numBpLayers;
            /*
             * Map all objects to a single broadphase layer,
             * which is simpler but less efficient:
             */
            int bpLayerAll = 0;
            result.mapObjectToBroadPhaseLayer(objLayerMoving, bpLayerAll);
            result.mapObjectToBroadPhaseLayer(objLayerNonMoving, bpLayerAll);
        }

        return result;
    }

    /**
     * Access the shape of the specified physics object.
     *
     * @param jpo the physics object (not null, unaffected)
     * @return the pre-existing object
     */
    private static ConstShape getShape(ConstJoltPhysicsObject jpo) {
        ConstShape result;
        if (jpo instanceof ConstBody) {
            result = ((ConstBody) jpo).getShape();
        } else if (jpo instanceof ConstCharacter) {
            result = ((ConstCharacter) jpo).getShape();
        } else if (jpo instanceof ConstCharacterVirtual) {
            result = ((ConstCharacterVirtual) jpo).getShape();
        } else if (jpo instanceof VehicleConstraint) {
            result = ((VehicleConstraint) jpo).getVehicleBody().getShape();
        } else {
            String className = jpo.getClass().getSimpleName();
            throw new IllegalArgumentException("class = " + className);
        }

        return result;
    }

    /**
     * Print basic library information to the specified stream during
     * initialization.
     *
     * @param printStream where to print the information (not null)
     */
    private static void printLibraryInfo(PrintStream printStream) {
        printStream.print("Jolt JNI version ");
        printStream.print(Jolt.versionString());
        printStream.print('-');

        String buildType = Jolt.buildType();
        printStream.print(buildType);

        if (Jolt.isDoublePrecision()) {
            printStream.print("Dp");
        } else {
            printStream.print("Sp");
        }

        printStream.println(" initializing...");
        printStream.println();
        printStream.flush();
    }
}
