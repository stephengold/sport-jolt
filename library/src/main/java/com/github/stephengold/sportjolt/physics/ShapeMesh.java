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
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import java.nio.FloatBuffer;

/**
 * An auto-generated mesh to visualize a collision shape.
 */
class ShapeMesh extends Mesh {
    // *************************************************************************
    // constructors

    /**
     * Auto-generate a mutable TriangleList mesh for the specified collision
     * shape.
     *
     * @param shape the shape from which to generate the mesh (not null,
     * unaffected)
     * @param meshResolution 0
     */
    ShapeMesh(ConstShape shape, int meshResolution) {
        super(Topology.TriangleList, getDebugTriangles(shape));
    }
    // *************************************************************************
    // private methods

    /**
     * Auto-generate triangle vertices to visualize the specified shape.
     *
     * @param shape the shape to use (not null, unaffected)
     * @return a new direct buffer with capacity a multiple of 9
     */
    private static FloatBuffer getDebugTriangles(ConstShape shape) {
        int numTriangles = shape.countDebugTriangles();
        int numFloats = 9 * numTriangles;
        FloatBuffer result = Jolt.newDirectFloatBuffer(numFloats);
        shape.copyDebugTriangles(result);

        return result;
    }
}
