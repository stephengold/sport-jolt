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
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.WrapFunction;
import org.joml.Vector3f;
import org.joml.Vector3fc;

/**
 * A simple graphics test: draw point sprites in model space.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class SpriteTest extends BaseApplication {
    // *************************************************************************
    // constructors

    /**
     * Instantiate the SpriteTest application.
     * <p>
     * This no-arg constructor was made explicit to avoid javadoc warnings from
     * JDK 18+.
     */
    public SpriteTest() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the SpriteTest application.
     *
     * @param arguments the array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        SpriteTest application = new SpriteTest();
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

        // a mesh composed of 2 points
        Vector3fc p0 = new Vector3f(0f, 0f, 0f);
        Vector3fc p1 = new Vector3f(0f, 1f, 0f);
        Mesh pointsMesh = new Mesh(Topology.PointList, p0, p1);

        TextureKey textureKey = createSpriteTexture();

        new Geometry(pointsMesh)
                .setColor(Constants.YELLOW)
                .setProgram("Unshaded/Sprite")
                .setTexture(textureKey);
    }
    // *************************************************************************
    // private methods

    /**
     * Generate a sprite texture with the "pin" shape.
     *
     * @return a new object
     */
    private static TextureKey createSpriteTexture() {
        String resourceName = "/Textures/shapes/pin.png";
        Filter magFilter = Filter.Linear;
        Filter minFilter = Filter.NearestMipmapLinear;
        WrapFunction wrapU = WrapFunction.ClampToEdge;
        WrapFunction wrapV = WrapFunction.ClampToEdge;
        boolean mipmaps = true;
        float maxAniso = 1f;
        TextureKey result = new TextureKey(
                "classpath://" + resourceName, magFilter, minFilter,
                wrapU, wrapV, mipmaps, FlipAxes.noFlip, maxAniso);

        return result;
    }
}
