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
package com.github.stephengold.sportjolt.mesh;

import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Topology;
import com.github.stephengold.sportjolt.Validate;

/**
 * A LineList mesh that renders a single line segment.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class LineMesh extends Mesh {
    // *************************************************************************
    // constructors

    /**
     * Instantiate a single line segment.
     *
     * @param start the location of the first endpoint (in mesh coordinates, not
     * null, unaffected)
     * @param end the location of the 2nd endpoint (in mesh coordinates, not
     * null, unaffected)
     */
    public LineMesh(Vec3 start, Vec3 end) {
        super(Topology.LineList, 2);
        Validate.nonNull(start, "start");
        Validate.nonNull(end, "end");

        super.setPositions(start.getX(), start.getY(), start.getZ(),
                end.getX(), end.getY(), end.getZ());
    }
}
