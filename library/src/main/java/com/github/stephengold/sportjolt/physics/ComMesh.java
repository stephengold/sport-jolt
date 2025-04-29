/*
 Copyright (c) 2025 Stephen Gold and Yanis Boudiaf

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

import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.readonly.ConstBody;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.VertexBuffer;
import java.nio.FloatBuffer;

/**
 * An auto-generated mesh to visualize a body's center of mass.
 */
class ComMesh extends Mesh {
    // *************************************************************************
    // fields

    /**
     * body being visualized
     */
    final private ConstBody body;
    /**
     * most recent location of the center of mass
     */
    final private RVec3 lastLocation = new RVec3();
    // *************************************************************************
    // constructors

    /**
     * Auto-generate a mutable PointList mesh for the specified body.
     *
     * @param body the body from which to generate the mesh (not null,
     * unaffected)
     */
    ComMesh(ConstBody body) {
        super(Topology.PointList, 1);

        this.body = body;

        VertexBuffer positions = super.createPositions();
        positions.setDynamic();

        update();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Update this mesh to match the body.
     */
    void update() {
        body.getCenterOfMassPosition(lastLocation);
        float x = lastLocation.x();
        float y = lastLocation.y();
        float z = lastLocation.z();

        FloatBuffer storeFloats = getPositionsData();
        storeFloats.put(0, x);
        storeFloats.put(1, y);
        storeFloats.put(2, z);

        setPositionsModified();
    }
}
