[The Sport-Jolt Project][project] implements
an [OpenGL]-based graphics engine
for [the Jolt-JNI 3-D physics library][joltjni].

It contains 2 subprojects:

1. library: the Sport-Jolt graphics engine (a single JVM runtime library,
   written in [Java])
2. java-apps: demos, non-automated test software, and other apps (in Java)

Complete source code is provided under
[a 3-clause BSD license][license].


<a name="toc"></a>

## Contents of this document

+ [About Sport Jolt](#about)
+ [Coding a Sport-Jolt application](#add)
+ [How to build and run Sport Jolt from source](#build)
+ [Conventions](#conventions)


<a name="about"></a>

## About Sport Jolt

Sport Jolt is a Simple Physics-ORienTed graphics engine written in Java 11.
In addition to [Jolt JNI][joltjni],
it uses [LWJGL], [Assimp], [GLFW], [JOML], [jSnapLoader], and [OpenGL].
It has been tested on [Linux], macOS, and Windows.

[Jump to the table of contents](#toc)


<a name="add"></a>

## How to add Sport Jolt to an existing project

Sport Jolt comes pre-built as a single library
that can be downloaded from Maven Central or GitHub.
However, the native-library dependencies are intentionally omitted
from Sport Jolt's POM
so developers can specify *which* Jolt-JNI and LWJGL natives should be used.

For projects built using [Maven] or [Gradle], it is
*not* sufficient to specify the
dependency on the Sport-Jolt Library.
You must also explicitly specify the native-library dependencies.

### Gradle-built projects

Add to the project’s "build.gradle" or "build.gradle.kts" file:

    repositories {
        mavenCentral()
    }
    dependencies {
        // JVM library:
        implementation("com.github.stephengold:sport-jolt:0.9.8")

        // Jolt-JNI native libraries:
        runtimeOnly("com.github.stephengold:jolt-jni-Linux64:3.0.0:DebugSp")
          // Jolt-JNI native libraries for other platforms could be added.

        // LWJGL native libraries:
        runtimeOnly("org.lwjgl:lwjgl:3.3.6:natives-linux")
        runtimeOnly("org.lwjgl:lwjgl-assimp:3.3.6:natives-linux")
        runtimeOnly("org.lwjgl:lwjgl-glfw:3.3.6:natives-linux")
        runtimeOnly("org.lwjgl:lwjgl-opengl:3.3.6:natives-linux")
          // LWJGL native libraries for other platforms could be added.
    }

For some older versions of Gradle,
it's necessary to replace `implementation` with `compile`.

### Coding a Sport-Jolt application

Every Sport-Jolt application should extend the `BasePhysicsApp` class,
which provides hooks for:

+ initializing the application,
+ creating and configuring the application's physics system,
+ populating the system with bodies and constraints, and
+ updating the system before each frame is rendered.

The graphics engine doesn't have a scene graph.
Instead, it maintains an internal list of renderable objects,
called *geometries*.
Instantiating a geometry automatically adds it to the list
and causes it to be rendered.

+ To visualize the world (system) coordinate axes,
  instantiate one or more `LocalAxisGeometry` objects.

By default, physics objects are not visualized.

+ To visualize the shape
  of a rigid body, character, or vehicle,
  invoke the `visualizeShape()` method on the object.
+ To visualize the local coordinate axes of a body, character, or vehicle,
  invoke the `visualizeAxes()` method on it.
+ To visualize the wheels of a vehicle,
  invoke the `visualizeWheels()` method on them.
+ To visualize the bounding box of a body, character, system, or vehicle,
  instantiate an `AabbGeometry` for the object.
+ To visualize the center of mass of a body,
  instantiate a `ComGeometry` for it.
+ To visualize a `Constraint`,
  instantiate a `ConstraintGeometry` for each end.
+ To visualize the faces of a soft body,
  instantiate a `FacesGeometry` for it.
+ To visualize the edges of a soft body,
  instantiate an `EdgesGeometry` for it.
+ To visualize the pinned vertices of a soft body,
  instantiate a `PinsGeometry` for it.
+ To visualize all vertices of a soft body,
  instantiate a `VerticesGeometry` for it.
+ To visualize the Cosserat rods of a soft body,
  invoke the `visualizeRods()` method on it.

[Jump to the table of contents](#toc)


<a name="build"></a>

## How to build and run Sport Jolt from source

[How to build and run the demos from source](https://stephengold.github.io/jolt-jni-docs/jolt-jni-en/English/demos.html#_how_to_build_and_run_the_demos_from_source)

Building demos automatically includes building the library.
After a successful build,
library artifacts will be found in "library/build/libs".

Once you’ve successfully built the library,
you can install it to your local Maven repository:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew install`
+ using Windows Command Prompt: `.\gradlew install`

At any time, you can restore the project to a pristine state:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew clean`
+ using Windows Command Prompt: `.\gradlew clean`

[Jump to the table of contents](#toc)


<a name="conventions"></a>

## Conventions

Package names begin with `com.github.stephengold.sportjolt`.

The source code and pre-built libraries are compatible with JDK 11.

3-D rotations, polygon windings, and coordinate axes
are right-handed/counter-clockwise unless otherwise noted.

Angles are quantified in *radians* unless otherwise noted.

The world coordinate system is assumed to be Z-forward, Y-up.

[Jump to the table of contents](#toc)


[adoptium]: https://adoptium.net/releases.html "Adoptium Project"
[assimp]: https://www.assimp.org/ "The Open Asset Importer Library"
[fish]: https://fishshell.com/ "Fish command-line shell"
[git]: https://git-scm.com "Git"
[glfw]: https://www.glfw.org "GLFW Library"
[gradle]: https://gradle.org "Gradle Project"
[java]: https://en.wikipedia.org/wiki/Java_(programming_language) "Java programming language"
[joltjni]: https://github.com/stephengold/jolt-jni "Jolt JNI Project"
[joml]: https://joml-ci.github.io/JOML "Java OpenGL Math Library"
[jsnaploader]: https://github.com/Electrostat-Lab/jSnapLoader "jSnapLoader Project"
[license]: https://github.com/stephengold/sport-jolt/blob/master/LICENSE "Sport-Jolt license"
[linux]: https://www.linux.com/what-is-linux "Linux"
[lwjgl]: https://www.lwjgl.org "Lightweight Java Game Library"
[maven]: https://maven.apache.org "Maven Project"
[opengl]: https://www.khronos.org/opengl "OpenGL API"
[project]: https://github.com/stephengold/sport-jolt "Sport-Jolt Project"
