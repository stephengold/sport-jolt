/*
 Copyright (c) 2025-2026 Stephen Gold and Yanis Boudiaf

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
package listclasses;

import com.github.stephengold.sportjolt.Utils;
import java.io.BufferedReader;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;

/**
 * Print the full names of Sport-Jolt's public classes to a text file.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class ListClasses {
    // *************************************************************************
    // constants

    /**
     * filesystem path to the output file
     */
    final private static String outputFilePath = "sport-jolt-classes.txt";
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private ListClasses() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the ListClasses application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        try {
            OutputStream fileStream = new FileOutputStream(outputFilePath);
            boolean autoFlush = true;
            PrintStream stream = new PrintStream(fileStream, autoFlush);
            printNamesTo(stream);

        } catch (IOException exception) {
            throw new RuntimeException(exception);
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Add the full names of all public classes in the specified package to the
     * specified set.
     *
     * @param nameSet the set to add to (not null)
     * @param packageName the name of the package (must start with
     * "com.github.stephengold.sportjolt")
     */
    private static void addPublicClassNames(
            Set<String> nameSet, String packageName) {
        Set<Class<?>> coreClasses = listClasses(packageName);
        for (Class<?> coreClass : coreClasses) {
            String simpleName = coreClass.getSimpleName();
            if (simpleName.isBlank()) {
                continue; // skip to the next Java class
            }
            int classModifiers = coreClass.getModifiers();
            if (!Modifier.isPublic(classModifiers)) {
                continue; // skip to the next Java class
            }
            nameSet.add(packageName + "." + simpleName);
        }
    }

    /**
     * List all classes in the specified Sport-Jolt package.
     *
     * @param packageName the name of the package (must start with
     * "com.github.stephengold.sportjolt")
     * @return a new collection
     */
    private static Set<Class<?>> listClasses(String packageName) {
        ClassLoader loader = Utils.class.getClassLoader();
        String resourcePath = packageName.replaceAll("[.]", "/");
        InputStream stream = loader.getResourceAsStream(resourcePath);
        if (stream == null) {
            System.err.println("resourcePath = " + resourcePath);
            System.exit(1);
        }
        InputStreamReader isr
                = new InputStreamReader(stream, StandardCharsets.UTF_8);
        BufferedReader reader = new BufferedReader(isr);

        Set<Class<?>> result = new HashSet<>();
        while (true) {
            try {
                String line = reader.readLine();
                if (line == null) {
                    break;

                } else if (line.endsWith(".class")) {
                    int dotPos = line.lastIndexOf('.');
                    String className = line.substring(0, dotPos);
                    try {
                        Class<?> clas
                                = Class.forName(packageName + "." + className);
                        result.add(clas);
                    } catch (ClassNotFoundException exception) {
                        System.err.println("className = " + className);
                        System.exit(0);
                    }
                }
            } catch (IOException exception) {
                throw new RuntimeException(exception);
            }
        }

        return result;
    }

    /**
     * Print the full names of Sport-Jolts's public classes to the specified
     * stream.
     *
     * @param stream the output stream to use (not null)
     */
    private static void printNamesTo(PrintStream stream) {
        Set<String> names = new TreeSet<>();

        // Enumerate Sport-Jolts's core and enum public classes:
        addPublicClassNames(names, "com.github.stephengold.sportjolt");
        addPublicClassNames(names, "com.github.stephengold.sportjolt.blend");
        addPublicClassNames(
                names, "com.github.stephengold.sportjolt.importers");
        addPublicClassNames(names, "com.github.stephengold.sportjolt.input");
        addPublicClassNames(names, "com.github.stephengold.sportjolt.mesh");
        addPublicClassNames(names, "com.github.stephengold.sportjolt.physics");

        for (String fullName : names) {
            stream.println(fullName);
        }
    }
}
