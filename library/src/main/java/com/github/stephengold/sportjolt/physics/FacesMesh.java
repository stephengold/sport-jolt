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

import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.SoftBodyMotionProperties;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.joltjni.readonly.ConstSoftBodySharedSettings;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.sportjolt.IndexBuffer;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.VertexBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 * An auto-generated TriangleList mesh to visualize the faces in a soft body.
 */
class FacesMesh extends Mesh {
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
    /**
     * copy vertex indices for the faces
     */
    final private IntBuffer copyIndices;
    // *************************************************************************
    // constructors

    /**
     * Auto-generate a mutable triangle mesh for the specified soft body.
     *
     * @param softBody the soft body from which to generate the mesh (not null,
     * alias created)
     */
    FacesMesh(ConstBody softBody) {
        super(Topology.TriangleList,
                BasePhysicsApp.getSharedSettings(softBody).countVertices());

        this.softBody = softBody;
        this.sharedSettings = BasePhysicsApp.getSharedSettings(softBody);

        // Create the VertexBuffer for vertex positions.
        VertexBuffer positions = super.createPositions();
        positions.setDynamic();

        // Create the VertexBuffer for vertex normals.
        VertexBuffer normals = super.createNormals();
        normals.setDynamic();

        // Create the IndexBuffer for vertex indices.
        int numFaces = sharedSettings.countFaces();
        int numIndices = vpt * numFaces;
        IndexBuffer indices = super.createIndices(numIndices);
        indices.setDynamic();

        // Create a buffer for copying indices.
        this.copyIndices = Jolt.newDirectIntBuffer(numIndices);

        update();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Update this Mesh to match the soft body.
     *
     * @return {@code true} if successful, otherwise {@code false}
     */
    boolean update() {
        int numVertices = sharedSettings.countVertices();
        if (numVertices != countVertices()) {
            return false;
        }
        int numFaces = sharedSettings.countFaces();
        if (numFaces != countTriangles()) {
            return false;
        }
        /*
         * Update the vertex positions from vertex locations in system
         * coordinates.
         */
        SoftBodyMotionProperties sbmp
                = (SoftBodyMotionProperties) softBody.getMotionProperties();
        RVec3Arg bodyPosition = softBody.getPosition();
        FloatBuffer storeFloats = getPositionsData();

        int saveBufferPosition = storeFloats.position();
        sbmp.putVertexLocations(bodyPosition, storeFloats);
        storeFloats.position(saveBufferPosition);
        setPositionsModified();

        // Update the index buffer from faces. TODO avoid copying indices
        copyIndices.clear();
        sharedSettings.putFaceIndices(copyIndices);
        IndexBuffer indices = getIndexBuffer();
        assert indices.position() == 0;
        for (int i = 0; i < vpt * numFaces; ++i) {
            int index = copyIndices.get(i);
            indices.put(i, index);
        }

        return true;
    }
}
