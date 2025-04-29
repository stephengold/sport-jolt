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
In addition to [Jolt JNI],
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
        implementation("com.github.stephengold:sport-jolt:0.9.1")

        // Libbulletjme native libraries:
        runtimeOnly("com.github.stephengold:jolt-jni-Linux64:0.9.10:DebugSp")
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
+ To visualize the bounding box of a body, character, or vehicle,
  instantiate an `AabbGeometry` for the object.
+ To visualize the center of mass of a body,
  instantiate a `ComGeometry` for it.
+ To visualize a `Constraint`,
  instantiate a `ConstraintGeometry` for each end.
+ To visualize the faces of a soft body,
  instantiate a `FacesGeometry` for it.
+ To visualize the edges of a soft body,
  instantiate a `LinksGeometry` for it.
+ To visualize the pins of a soft body,
  instantiate a `PinsGeometry` for it.

[Jump to the table of contents](#toc)


<a name="build"></a>

## How to build and run Sport Jolt from source

### Initial build

1. Install a [Java Development Kit (JDK)][adoptium],
   if you don't already have one.
2. Point the `JAVA_HOME` environment variable to your JDK installation:
   (In other words, set it to the path of a directory/folder
   containing a "bin" that contains a Java executable.
   That path might look something like
   "C:\Program Files\Eclipse Adoptium\jdk-17.0.3.7-hotspot"
   or "/usr/lib/jvm/java-17-openjdk-amd64/" or
   "/Library/Java/JavaVirtualMachines/zulu-17.jdk/Contents/Home" .)
  + using Bash or Zsh: `export JAVA_HOME="` *path to installation* `"`
  + using [Fish]: `set -g JAVA_HOME "` *path to installation* `"`
  + using Windows Command Prompt: `set JAVA_HOME="` *path to installation* `"`
  + using PowerShell: `$env:JAVA_HOME = '` *path to installation* `'`
3. Download and extract the Sport-Jolt source code from GitHub:
  + using [Git]:
    + `git clone https://github.com/stephengold/sport-jolt.git`
    + `cd sport-jolt`
    + `git checkout -b latest 0.9.1`
4. (optional) Edit the "gradle.properties" file to configure the build.
5. Run the [Gradle] wrapper:
  + using Bash or Fish or PowerShell or Zsh: `./gradlew build`
  + using Windows Command Prompt: `.\gradlew build`

After a successful build,
[Maven] artifacts will be found in "library/build/libs".

You can install the artifacts to your local Maven repository:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew install`
+ using Windows Command Prompt: `.\gradlew install`

### Demos

Three demonstration applications (in Java) are included:

+ NewtonsCradle (a "Newton's cradle" toy)
  + using Bash or Fish or PowerShell or Zsh: `./gradlew :java-apps:NewtonsCradle`
  + using Windows Command Prompt: `.\gradlew :java-apps:NewtonsCradle`
  + Press Pause or "." to start or pause the physics.
  + Press 1/2/3/4/5 to reinitialize with a different number of balls.
  + Press W/A/S/D/Q/Z to move the camera.

+ Pachinko (2-D simulation of a simple Pachinko machine)
  + using Bash or Fish or PowerShell or Zsh: `./gradlew :java-apps:Pachinko`
  + using Windows Command Prompt: `.\gradlew :java-apps:Pachinko`
  + Press Pause or "." to pause or resume the physics.
  + Press 4/5/6/7/8/9 to restart the simulation with a different pin layout.

+ ThousandCubes
  (drop 1000 cubes onto a horizontal surface and shoot balls at them)
  + using Bash or Fish or PowerShell or Zsh:
    `./gradlew -Passertions=false -Pbtf=ReleaseSp :java-apps:ThousandCubes`
  + using Windows Command Prompt:
    `.\gradlew -Passertions=false -Pbtf=ReleaseSp :java-apps:ThousandCubes`
  + Press E to shoot.
  + Press Pause or "." to pause or resume the physics.
  + Press W/A/S/D/Q/Z to move the camera.

### Cleanup

You can restore the project to a pristine state:
+ using Bash or Fish or PowerShell or Zsh: `./gradlew clean`
+ using Windows Command Prompt: `.\gradlew clean`

[Jump to the table of contents](#toc)


<a name="conventions"></a>

## Conventions

Package names begin with `com.github.stephengold.sportjolt`.

The source code and pre-built libraries are compatible with JDK 11.

Rotation signs, polygon windings, and 3-D coordinate axes
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
