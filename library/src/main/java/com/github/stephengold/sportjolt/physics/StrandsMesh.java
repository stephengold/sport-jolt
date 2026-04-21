/*
 Copyright (c) 2026 Stephen Gold and Yanis Boudiaf

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

import com.github.stephengold.joltjni.Hair;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.readonly.ConstHairSettings;
import com.github.stephengold.sportjolt.IndexBuffer;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.VertexBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 * An auto-generated LineList mesh to visualize the strands in a hair
 * simulation.
 */
class StrandsMesh extends Mesh {
    // *************************************************************************
    // fields

    /**
     * simulation being visualized
     */
    final private Hair hair;
    // *************************************************************************
    // constructors

    /**
     * Auto-generate a mutable line mesh for the specified simulation.
     *
     * @param hair the simulation from which to generate the mesh (not null,
     * alias created)
     */
    StrandsMesh(Hair hair) {
        super(Topology.LineList, hair.getHairSettings().countRenderVertices());

        this.hair = hair;

        // Create the VertexBuffer for vertex positions:
        VertexBuffer positions = super.createPositions();
        positions.setDynamic();

        // Create the IndexBuffer for vertex indices:
        ConstHairSettings settings = hair.getHairSettings();
        int numEdges = settings.countRenderVertices()
                - settings.countRenderStrands();
        int numIndices = vpe * numEdges;
        IndexBuffer indices = super.createIndices(numIndices);

        // Fill the index buffer from strands:
        IntBuffer copyIndices = Jolt.newDirectIntBuffer(numIndices);
        settings.putEdgeIndices(copyIndices);
        for (int ii = 0; ii < vpe * numEdges; ++ii) {
            int vertexIndex = copyIndices.get(ii);
            indices.put(ii, vertexIndex);
        }

        update();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Update the positions data to match the simulation.
     *
     * @return {@code true} if successful, otherwise {@code false}
     */
    boolean update() {
        FloatBuffer positions = getPositionsData();
        assert positions.position() == 0;

        hair.lockReadBackBuffers();
        hair.putRenderPositionsWorld(positions);
        hair.unlockReadBackBuffers();

        positions.clear();
        setPositionsModified();

        return true;
    }
}
