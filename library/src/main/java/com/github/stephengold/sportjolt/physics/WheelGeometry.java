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

import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.VehicleConstraint;
import com.github.stephengold.joltjni.Wheel;
import com.github.stephengold.joltjni.readonly.ConstWheelSettings;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.Geometry;
import com.github.stephengold.sportjolt.Validate;
import com.github.stephengold.sportjolt.mesh.WheelMesh;

/**
 * Visualize one of the wheels of a vehicle.
 */
public class WheelGeometry extends Geometry {
    // *************************************************************************
    // constants

    /**
     * "right" direction in the wheel's model space
     */
    final private static Vec3Arg right = Vec3.sAxisX();
    /**
     * "up" direction in the wheel's model space
     */
    final private static Vec3Arg up = Vec3.sAxisY();
    // *************************************************************************
    // fields

    /**
     * index of the wheel to visualize (&ge;0)
     */
    final private int wheelIndex;
    /**
     * most recent orientation of the body
     */
    final private Quat lastOrientation = new Quat();
    /**
     * most recent location of the body
     */
    final private RVec3 lastLocation = new RVec3();
    /**
     * vehicle to visualize
     */
    final private VehicleConstraint vehicle;
    /**
     * reusable mesh, allocated lazily
     */
    private static WheelMesh mesh;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a Geometry to visualize the specified wheel of the specified
     * vehicle.
     *
     * @param vehicle the vehicle (not null, alias created)
     * @param wheelIndex which wheel (&ge;0)
     */
    public WheelGeometry(VehicleConstraint vehicle, int wheelIndex) {
        Validate.nonNull(vehicle, "vehicle");
        Validate.nonNegative(wheelIndex, "wheel index");

        this.vehicle = vehicle;
        this.wheelIndex = wheelIndex;

        if (mesh == null) {
            mesh = new WheelMesh();
        }
        super.setMesh(mesh);

        super.setProgram("Unshaded/Monochrome");
        BaseApplication.makeVisible(this);
    }
    // *************************************************************************
    // Geometry methods

    /**
     * Update properties based on the wheel and then render.
     */
    @Override
    public void updateAndRender() {
        updateTransform();
        super.updateAndRender();
    }

    /**
     * Test whether the wheel has been removed from the specified PhysicsSystem.
     *
     * @param system the system to test (not null, unaffected)
     * @return {@code true} if removed, otherwise {@code false}
     */
    @Override
    public boolean wasRemovedFrom(PhysicsSystem system) {
        if (system.containsConstraint(vehicle)) {
            int numWheels = vehicle.countWheels();
            if (wheelIndex < numWheels) {
                return false;
            }
        }

        return true;
    }
    // *************************************************************************
    // private methods

    /**
     * Update the mesh-to-world transform.
     */
    private void updateTransform() {
        vehicle.getWheelPositionAndRotation(
                wheelIndex, right, up, lastLocation, lastOrientation);
        setLocation(lastLocation);
        setOrientation(lastOrientation);

        Wheel wheel = vehicle.getWheel(wheelIndex);
        ConstWheelSettings settings = wheel.getSettings();
        float radius = settings.getRadius();
        setScale(radius);
    }
}
