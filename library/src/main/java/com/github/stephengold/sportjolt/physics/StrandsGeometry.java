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
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Constants;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Mesh;
import com.github.stephengold.sportjolt.Validate;

/**
 * Visualize the strands of a hair simulation.
 */
public class StrandsGeometry extends Geometry {
    // *************************************************************************
    // fields

    /**
     * hair simulation to visualize
     */
    final private Hair hair;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the specified hair simulation and
     * make the Geometry visible.
     *
     * @param hair the simulation to visualize (not null, alias created)
     */
    public StrandsGeometry(Hair hair) {
        Validate.nonNull(hair, "hair");

        this.hair = hair;
        super.setColor(Constants.WHITE);

        Mesh mesh = new StrandsMesh(hair);
        super.setMesh(mesh);

        BaseApplication.makeVisible(this);
    }
    // *************************************************************************
    // Geometry methods

    /**
     * Update properties based on the simulation and then render.
     */
    @Override
    public void updateAndRender() {
        updateMesh();
        super.updateAndRender();
    }

    /**
     * Test whether the hair has been removed from the specified
     * {@code PhysicsSystem}. In general there's no way to test this, so the
     * application will have to explicitly remove the geometry.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        return false;
    }
    // *************************************************************************
    // private methods

    /**
     * Update the Mesh.
     */
    private void updateMesh() {
        StrandsMesh mesh = (StrandsMesh) getMesh();
        boolean success = mesh.update();
        if (!success) {
            mesh = new StrandsMesh(hair);
            super.setMesh(mesh);
        }
    }
}
