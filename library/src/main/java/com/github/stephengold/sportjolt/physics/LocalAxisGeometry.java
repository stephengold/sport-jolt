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
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstCharacter;
import com.github.stephengold.joltjni.readonly.ConstCharacterVirtual;
import com.github.stephengold.joltjni.readonly.ConstJoltPhysicsObject;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.mesh.ArrowMesh;
import org.joml.Vector4fc;

/**
 * Visualize one of the local axes of a physics object or else a "floating"
 * arrow.
 */
public class LocalAxisGeometry extends Geometry {
    // *************************************************************************
    // constants

    /**
     * map axis indices to colors
     */
    final private static Vector4fc[] colors = {
        Constants.RED, // X
        Constants.GREEN, // Y
        Constants.BLUE // Z
    };
    // *************************************************************************
    // fields

    /**
     * physics object to visualize
     */
    final private ConstJoltPhysicsObject jpo;
    /**
     * length of the axis (in world units)
     */
    final private float length;
    /**
     * most recent orientation of the physics object
     */
    final private Quat lastOrientation = new Quat();
    /**
     * most recent location of the physics object
     */
    final private RVec3 lastLocation = new RVec3();
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the specified local axis of the
     * specified physics object and make the Geometry visible.
     *
     * @param jpo the physics object (alias created) or null for a "floating"
     * axis
     * @param axisIndex which axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @param length the length of the axis (in world units, &ge;0)
     */
    public LocalAxisGeometry(
            ConstJoltPhysicsObject jpo, int axisIndex, float length) {
        super();
        Validate.axisIndex(axisIndex, "axisIndex");
        Validate.nonNegative(length, "length");
        assert jpo == null || jpo instanceof ConstBody
                || jpo instanceof ConstCharacter
                || jpo instanceof ConstCharacterVirtual;

        this.jpo = jpo;
        this.length = length;

        Vector4fc color = colors[axisIndex];
        super.setColor(color);

        Mesh mesh = ArrowMesh.getMesh(axisIndex);
        super.setMesh(mesh);

        super.setProgram("Unshaded/Monochrome");

        BaseApplication.makeVisible(this);
    }
    // *************************************************************************
    // Geometry methods

    /**
     * Update properties based on the physics object and then render.
     */
    @Override
    public void updateAndRender() {
        updateTransform();
        super.updateAndRender();
    }

    /**
     * Test whether the physics object has been removed from the specified
     * PhysicsSystem.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        boolean result;
        if (jpo == null) {
            result = false;

        } else if (jpo instanceof ConstBody) {
            ConstBody body = (ConstBody) jpo;
            BodyInterface bi = system.getBodyInterface();
            int bodyId = body.getId();
            result = !bi.isAdded(bodyId);

        } else if (jpo instanceof ConstCharacter) {
            ConstCharacter character = (ConstCharacter) jpo;
            BodyInterface bi = system.getBodyInterface();
            int bodyId = character.getBodyId();
            result = !bi.isAdded(bodyId);

        } else if (jpo instanceof ConstCharacterVirtual) {
            /*
             * In general there's no way to test this, so the
             * application will have to explicitly remove the geometry.
             */
            result = false;

        } else {
            throw new IllegalStateException(jpo.getClass().getSimpleName());
        }

        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Update the mesh-to-world transform.
     */
    private void updateTransform() {
        if (jpo == null) {
            // no change to transform

        } else if (jpo instanceof ConstBody) {
            ConstBody body = (ConstBody) jpo;

            body.getPositionAndRotation(lastLocation, lastOrientation);
            setLocation(lastLocation);
            setOrientation(lastOrientation);

        } else if (jpo instanceof ConstCharacter) {
            ConstCharacter character = (ConstCharacter) jpo;

            character.getPositionAndRotation(
                    lastLocation, lastOrientation, false);
            setLocation(lastLocation);
            setOrientation(lastOrientation);

        } else if (jpo instanceof ConstCharacterVirtual) {
            ConstCharacterVirtual character = (ConstCharacterVirtual) jpo;

            character.getPositionAndRotation(lastLocation, lastOrientation);
            setLocation(lastLocation);
            setOrientation(lastOrientation);

        } else {
            throw new IllegalStateException(jpo.getClass().getSimpleName());
        }

        setScale(length);
    }
}
