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
package com.github.stephengold.sportjolt.javaapp.test;

import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.UvsOption;
import com.github.stephengold.sportjolt.mesh.RectangleMesh;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;

/**
 * A simple graphics test: generate texture coordinates using a linear function.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class RainbowTest extends BaseApplication {
    // *************************************************************************
    // fields

    /**
     * multi-colored square in clipspace
     */
    private static Geometry squareGeometry;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the RainbowTest application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public RainbowTest() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the RainbowTest application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        RainbowTest application = new RainbowTest();
        application.start();
    }
    // *************************************************************************
    // BaseApplication methods

    /**
     * Callback invoked after the main update loop terminates.
     */
    @Override
    protected void cleanUp() {
        // do nothing
    }

    /**
     * Initialize the application.
     */
    @Override
    protected void initialize() {
        setVsync(true);
        setBackgroundColor(Constants.SKY_BLUE);

        float radius = 0.5f; // as a multiple of half the window size
        Mesh squareMesh
                = new RectangleMesh(-radius, radius, -radius, radius, 1f);
        squareMesh.generateUvs(UvsOption.Linear,
                new Vector4f(1f, 0f, 0f, 0.5f),
                new Vector4f(0f, 1f, 0f, 0.5f));

        squareGeometry = new Geometry(squareMesh)
                .setProgram("Unshaded/Clipspace/RainbowTest");
    }

    /**
     * Callback invoked during each iteration of the main update loop.
     */
    @Override
    protected void render() {
        updateScales();
        super.render();
    }
    // *************************************************************************
    // private methods

    /**
     * Scale the Geometry so it will render as a square, regardless of the
     * window's aspect ratio.
     */
    private void updateScales() {
        float aspectRatio = aspectRatio();
        float yScale = Math.min(1f, aspectRatio);
        float xScale = yScale / aspectRatio;
        Vector3fc newScale = new Vector3f(xScale, yScale, 1f);

        squareGeometry.setScale(newScale);
    }
}
