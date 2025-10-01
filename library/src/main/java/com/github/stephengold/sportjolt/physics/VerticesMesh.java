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

import com.github.stephengold.joltjni.SoftBodyMotionProperties;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstSoftBodySharedSettings;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.VertexBuffer;
import java.nio.FloatBuffer;

/**
 * An auto-generated mesh to visualize the vertices in a soft body.
 */
class VerticesMesh extends Mesh {
    // *************************************************************************
    // fields

    /**
     * soft body being visualized
     */
    final private ConstBody softBody;
    /**
     * shared settings of the body
     */
    final ConstSoftBodySharedSettings sharedSettings;
    // *************************************************************************
    // constructors

    /**
     * Auto-generate a mutable PointList mesh for the specified soft body.
     *
     * @param softBody the soft body from which to generate the mesh (not null,
     * unaffected)
     */
    VerticesMesh(ConstBody softBody) {
        super(Topology.PointList,
                BasePhysicsApp.getSharedSettings(softBody).countVertices());

        this.softBody = softBody;
        this.sharedSettings = BasePhysicsApp.getSharedSettings(softBody);

        VertexBuffer positions = super.createPositions();
        positions.setDynamic();

        updatePositions();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Update this mesh to match the soft body.
     *
     * @return {@code true} if successful, otherwise {@code false}
     */
    boolean update() {
        int numVertices = sharedSettings.countVertices();
        if (numVertices != countVertices()) {
            return false;
        } else {
            updatePositions();
            return true;
        }
    }
    // *************************************************************************
    // private methods

    /**
     * Update the vertex positions in system/world coordinates.
     */
    private void updatePositions() {
        SoftBodyMotionProperties properties
                = (SoftBodyMotionProperties) softBody.getMotionProperties();
        RVec3Arg bodyPosition = softBody.getPosition();
        FloatBuffer storeFloats = getPositionsData();

        int saveBufferPosition = storeFloats.position();
        properties.putVertexLocations(bodyPosition, storeFloats);
        storeFloats.position(saveBufferPosition);
        setPositionsModified();
    }
}
