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

import com.github.stephengold.joltjni.BodyInterface;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.enumerate.EBodyType;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Filter;
import com.github.stephengold.sportjolt.FlipAxes;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.TextureKey;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.WrapFunction;

/**
 * Visualize the vertices of a soft body.
 */
public class VerticesGeometry extends Geometry {
    // *************************************************************************
    // fields

    /**
     * soft body to visualize
     */
    final private ConstBody softBody;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the specified soft body and make the
     * Geometry visible.
     *
     * @param softBody the body to visualize (not null, alias created)
     */
    public VerticesGeometry(ConstBody softBody) {
        Validate.nonNull(softBody, "soft body");

        assert softBody.getBodyType() == EBodyType.SoftBody;
        this.softBody = softBody;
        super.setColor(Constants.RED);

        Mesh mesh = new VerticesMesh(softBody);
        super.setMesh(mesh);

        super.setPointSize(3f);
        super.setProgram("Unshaded/Sprite");

        TextureKey textureKey = createSpriteTexture();
        super.setTexture(textureKey);

        BaseApplication.makeVisible(this);
    }
    // *************************************************************************
    // Geometry methods

    /**
     * Update properties based on the body and then render.
     */
    @Override
    public void updateAndRender() {
        updateMesh();
        super.updateAndRender();
    }

    /**
     * Test whether the body has been removed from the specified PhysicsSystem.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        BodyInterface bi = system.getBodyInterface();
        int bodyId = softBody.getId();
        boolean result = !bi.isAdded(bodyId);

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Generate a sprite texture with the "solid circle" shape.
     *
     * @return a new object
     */
    private static TextureKey createSpriteTexture() {
        String resourceName = "/Textures/shapes/solid%20circle.png";
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

    /**
     * Update the Mesh.
     */
    private void updateMesh() {
        VerticesMesh mesh = (VerticesMesh) getMesh();
        boolean success = mesh.update();
        if (!success) {
            mesh = new VerticesMesh(softBody);
            setMesh(mesh);
        }
    }
}
