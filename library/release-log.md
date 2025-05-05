# release log for the Sport-Jolt Library

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
