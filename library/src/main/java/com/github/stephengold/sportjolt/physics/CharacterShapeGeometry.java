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
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.readonly.ConstCharacter;
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Validate;
import org.joml.Vector4fc;

/**
 * Visualize the shape of a Jolt Character.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class CharacterShapeGeometry extends Geometry {
    // *************************************************************************
    // fields

    /**
     * {@code true} to automatically update the color based on properties of the
     * character, {@code false} for custom color
     */
    private boolean automaticColor = true;
    /**
     * character to visualize
     */
    final private ConstCharacter character;
    /**
     * most recent orientation of the character
     */
    final private Quat lastOrientation = new Quat();
    /**
     * most recent location of the character
     */
    final private RVec3 lastLocation = new RVec3();
    /**
     * auxiliary data used to generate the current mesh
     */
    private ShapeSummary summary;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the specified character and make the
     * Geometry visible.
     *
     * @param character the character to visualize (not null, alias created)
     * @param meshingStrategy how to generate meshes (not null)
     */
    CharacterShapeGeometry(
            ConstCharacter character, MeshingStrategy meshingStrategy) {
        Validate.nonNull(character, "character");
        Validate.nonNull(meshingStrategy, "meshing strategy");

        this.character = character;

        ConstShape shape = character.getShape();
        this.summary = new ShapeSummary(shape, meshingStrategy);
        Mesh mesh = BasePhysicsApp.meshForShape(shape, summary);
        super.setMesh(mesh);

        BaseApplication.makeVisible(this);
    }
    // *************************************************************************
    // Geometry methods

    /**
     * Alter the base color and disable automatic updating of it.
     *
     * @param r the desired red component (in the Linear colorspace)
     * @param g the desired green component (in the Linear colorspace)
     * @param b the desired blue component (in the Linear colorspace)
     * @return the (modified) current geometry (for chaining)
     */
    @Override
    public CharacterShapeGeometry setColor(float r, float g, float b) {
        this.automaticColor = false;
        super.setColor(r, g, b);

        return this;
    }

    /**
     * Alter the base color and disable automatic updating of it.
     *
     * @param newColor the desired color (in the Linear colorspace, not null,
     * unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    @Override
    public CharacterShapeGeometry setColor(Vector4fc newColor) {
        this.automaticColor = false;
        super.setColor(newColor);

        return this;
    }

    /**
     * Update properties based on the character and then render.
     */
    @Override
    public void updateAndRender() {
        updateColor();
        updateMesh();
        updateTransform();

        super.updateAndRender();
    }

    /**
     * Test whether the character has been removed from the specified
     * {@code PhysicsSystem}.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        BodyInterface bi = system.getBodyInterface();
        int bodyId = character.getBodyId();
        boolean result = !bi.isAdded(bodyId);

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Update the color.
     */
    private void updateColor() {
        if (automaticColor) {
            if (character.isSupported()) {
                super.setColor(Constants.BROWN);
            } else {
                super.setColor(Constants.PINK);
            }
        }
    }

    /**
     * Update the Mesh.
     */
    private void updateMesh() {
        ConstShape shape = character.getShape();
        if (!summary.matches(shape)) {
            MeshingStrategy strategy = summary.meshingStrategy();
            this.summary = new ShapeSummary(shape, strategy);
            Mesh mesh = BasePhysicsApp.meshForShape(shape, summary);
            super.setMesh(mesh);
        }
    }

    /**
     * Update the mesh-to-world transform.
     */
    private void updateTransform() {
        character.getPositionAndRotation(lastLocation, lastOrientation, false);
        setLocation(lastLocation);
        setOrientation(lastOrientation);
        setScale(1f);
    }
}
