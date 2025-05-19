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
import com.github.stephengold.sportjolt.Filter;
import com.github.stephengold.sportjolt.FlipAxes;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.TextureKey;
import com.github.stephengold.sportjolt.UvsOption;
import com.github.stephengold.sportjolt.WrapFunction;
import com.github.stephengold.sportjolt.blend.OverOp;
import com.github.stephengold.sportjolt.mesh.RectangleMesh;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;

/**
 * A simple graphics test: load and display a PNG image in clipspace.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class TextureTest extends BaseApplication {
    // *************************************************************************
    // fields

    /**
     * textured square in clipspace
     */
    private static Geometry squareGeometry;
    /**
     * OpenGL name of the transparent texture
     */
    private static int redBarTextureName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate the TextureTest application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public TextureTest() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the TextureTest application.
     *
     * @param arguments the array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        TextureTest application = new TextureTest();
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
                new Vector4f(0f, -1f, 0f, 0.5f) // flip the vertical axis
        );

        squareGeometry = new Geometry(squareMesh)
                .setProgram("Unshaded/Clipspace/Texture");

        String resourceName = "/Textures/TextureTest.png";
        TextureKey textureKey = new TextureKey("classpath://" + resourceName);
        squareGeometry.setTexture(textureKey);

        // Load a transparent image as a texture, flipping the Y axis:
        resourceName = "/Textures/RedBar.png";
        textureKey = new TextureKey("classpath://" + resourceName,
                Filter.Linear, Filter.NearestMipmapLinear,
                WrapFunction.Repeat, WrapFunction.Repeat,
                true, FlipAxes.flipY, 1f);
        redBarTextureName = textureKey.textureName();
    }

    /**
     * Callback invoked during each iteration of the main update loop.
     */
    @Override
    protected void render() {
        updateScales();
        super.render();
        blendTexture(redBarTextureName, new OverOp());
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
