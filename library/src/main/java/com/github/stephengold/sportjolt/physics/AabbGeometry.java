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
import com.github.stephengold.joltjni.readonly.ConstAaBox;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstCharacter;
import com.github.stephengold.joltjni.readonly.ConstCharacterVirtual;
import com.github.stephengold.joltjni.readonly.ConstJoltPhysicsObject;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.mesh.BoxOutlineMesh;

/**
 * Visualize the axis-aligned bounding box of a physics object.
 */
public class AabbGeometry extends Geometry {
    // *************************************************************************
    // fields

    /**
     * physics object to visualize
     */
    final private ConstJoltPhysicsObject jpo;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the axis-aligned bounding box of the
     * specified physics object and make the Geometry visible.
     *
     * @param jpo the physics object (not null, alias created)
     */
    public AabbGeometry(ConstJoltPhysicsObject jpo) {
        super();
        Validate.nonNull(jpo, "physics object");
        if (jpo instanceof ConstBody || jpo instanceof ConstCharacter
                || jpo instanceof ConstCharacterVirtual) {
            // do nothing
        } else {
            throw new IllegalStateException(jpo.getClass().getSimpleName());
        }
        this.jpo = jpo;

        Mesh mesh = BoxOutlineMesh.getMesh();
        super.setProgram("Unshaded/Monochrome");

        super.setMesh(mesh);

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
     * {@code PhysicsSystem}.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        BodyInterface bi = system.getBodyInterface();
        boolean result;
        if (jpo instanceof ConstBody) {
            ConstBody body = (ConstBody) jpo;
            int bodyId = body.getId();
            result = !bi.isAdded(bodyId);

        } else if (jpo instanceof ConstCharacter) {
            ConstCharacter character = (ConstCharacter) jpo;
            int bodyId = character.getBodyId();
            result = !bi.isAdded(bodyId);

        } else if (jpo instanceof ConstCharacterVirtual) {
            /*
             * In general there's no way to test this, so the
             * application will have to explicitly remove the Geometry.
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
        ConstAaBox bbox;
        if (jpo instanceof ConstBody) {
            ConstBody body = (ConstBody) jpo;
            bbox = body.getWorldSpaceBounds();

        } else if (jpo instanceof ConstCharacter) {
            ConstCharacter character = (ConstCharacter) jpo;
            bbox = character.getTransformedShape().getWorldSpaceBounds();

        } else if (jpo instanceof ConstCharacterVirtual) {
            ConstCharacterVirtual character = (ConstCharacterVirtual) jpo;
            bbox = character.getTransformedShape().getWorldSpaceBounds();

        } else {
            throw new IllegalStateException(jpo.getClass().getSimpleName());
        }

        Vec3Arg center = bbox.getCenter(); // garbage
        setLocation(center);

        Vec3Arg extent = bbox.getExtent(); // garbage
        setScale(extent);
    }
}
