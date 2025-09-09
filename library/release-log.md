# release log for the Sport-Jolt Library

## Version 0.9.10 released on 8 September 2025

Updated the Jolt-JNI library to v3.1.0 .

## Version 0.9.9 released on 31 August 2025

Updated the Jolt-JNI library to v3.0.1 .

## Version 0.9.8 released on 30 August 2025

+ Added methods:
  + `BaseApplication.start(String)`
  + `BasePhysicsApp.visualizeBodyShape()`
  + `BasePhysicsApp.visualizeRods()`
+ Added the capability to configure logging and/or a cleaner during startup.
+ Updated the Jolt-JNI library to v3.0.0 .

## Version 0.9.7 released on 10 August 2025

+ Added classes:
  + `RodsGeometry`
  + `RodsMesh`
  + `VerticesGeometry`
  + `VerticesMesh`

+ Made more setters chainable.
+ Added alternative signtures for `setColor()` methods.
+ Added the `BasePhysicsApp.getPhysicsSystem()` method
+ More specific return types.
+ Publicized 4 protected visualization methods in `BasePhysicsApp`.
+ Updated the Jolt-JNI library to v2.2.0 .

## Version 0.9.6 released on 25 July 2025

+ Added constructors and methods:
  + `BaseApplication.setBackgroundColor(float, float, float, float)`
  + `BaseApplication.setLightColor(Vector4fc)`
  + `BoxMesh(float, float, float)`
  + `RigidBodyShapeGeometry.getBodyId()`
  + `RigidBodyShapeGeometry.getBodyVa()`
  + `ShapeSummary(ConstShape, String)`
  + `Utils.toJoltQuaternion(Quaternionfc)`
  + `Utils.toJoltQuaternion(Quaternionfc, Quat)`

+ Publicized the `BasePhysicsApp.meshForShape()` method.
+ Tuned the `PhysicsSystem` sizing heuristics.
+ Updated the Jolt-JNI library to v2.1.0 .

## Version 0.9.5 released on 30 June 2025

Updated the Jolt-JNI library to v2.0.1 and the OSHI library to v6.8.2 .

## Version 0.9.4 released on 18 May 2025

+ API changes:
  + Renamed the `LinksMesh` and `LinksGeometry` classes.
  + Changed the argument type of `Camera.setLocation()`.
  + Changed the return type of `InputManager.destroy()`.
  + Use JOML vectors for efficiency in the `DividedLine` class.

+ Bugfixes:
  + potential for incorrect setting due to `int` overflow in `BasePhysicsApp`

+ Began handling soft bodies in `BasePhysicsApp.visualizeShape()`.
+ Began visualizing heightfield shapes with a checkerboard texture.
+ Updated the Jolt-JNI library to v1.0.0 .

## Version 0.9.3 released on 8 May 2025

+ API change:
  + de-publicize 6 methods in the `BasePhysicsApp` class

+ Bugfixes:
  + `createSystem()` with maxBodies=2 triggers a native assertion
  + failing cast in `BasePhysicsApp.visualizeShape()`
  + `CharacterVirtualShapeGeometry` without a body is never visualized

+ Add the `getTempAllocator()` method to the `BasePhysicsApp` class.
+ Handle characters better in `LocalAxisGeometry`.
+ Add support in `AabbGeometry` to visualize the bounds
  of a `CharacterVirtual` or a `PhysicsSystem`.

## Version 0.9.2 released on 5 May 2025

+ Added 4 methods:
  + `BaseApplication.initialWindowTitle()`
  + `BasePhysicsApp.createSystem(int maxBodies, int numBpLayers)`
    to create a generic `PhysicsSystem`
  + `BasePhysicsApp.totalPhysicsNanos()`
    to report cumulative wall-clock time spent in physics simulation
  + `BasePhysicsApp.totalSimulatedTime()`
    to report total time simulated

## Version 0.9.1 released on 29 April 2025

+ Improved the performance of the `ThousandCubes` demo (see issue #1):
  + Redesigned the `Geometry` and `Transform` classes. (API changes)
+ Replaced `Utils.toHeightArray()` with `toHeightBuffer()`. (API change)

+ bugfix: `ShapeSummary` is vulnerable to re-use of the shape's virtual address

+ Added public classes:
  + `ComGeometry`
  + `ComMesh`
+ Added public methods:
  + `Utils.modulo()` for `int`
  + `Utils.toJomlQuaternion()`
+ Publicized 4 methods in the `Vertex` class.
+ Added 7 more shape textures for use in sprites.
+ Added an option to `ConstraintGeometry` to visualize both pivots.
+ Enhanced `BasePhysicsApp.cleanUp()` to simplify reuse of the `PhysicsSystem`:
  + Began removing all constraints and destroying all bodies.
  + Began hiding all geometries.
+ Suppressed culling of back faces when visualizing `TriangleShape`.
+ Switched to faceted meshes for visualizing `Cylinder` and `TaperedCylinder`.
+ Specified clean extraction of native libraries.
+ Improved reporting of OpenGL errors.
+ Switched to a more flexible implementation of `TempAllocator`.
+ Updated the Jolt-JNI library to v0.9.10 and the OSHI library to v6.8.1 .

## Version 0.9.0 released on 31 March 2025

Initial release, using code adapted from the SPORT project.
