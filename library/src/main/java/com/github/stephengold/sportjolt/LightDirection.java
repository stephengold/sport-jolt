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
package com.github.stephengold.sportjolt;

import com.github.stephengold.joltjni.Vec3;

/**
 * Provide the direction to the distant light, for use in shaders.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class LightDirection extends GlobalUniform {
    // *************************************************************************
    // fields

    /**
     * current direction (in world coordinates)
     */
    final private static Vec3 value = new Vec3(1f, 3f, 2f).normalized();
    // *************************************************************************
    // constructors

    /**
     * Instantiate this uniform.
     */
    LightDirection() {
        super("LightDirection_worldspace");
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Alter the direction to the distant light.
     *
     * @param x the X component of the desired direction
     * @param y the Y component of the desired direction
     * @param z the Z component of the desired direction
     */
    static void set(float x, float y, float z) {
        value.set(x, y, z);
        value.normalizeInPlace();
    }
    // *************************************************************************
    // GlobalUniform methods

    /**
     * Send the current value to the specified program.
     *
     * @param program the program to update (not null)
     */
    @Override
    void sendValueTo(ShaderProgram program) {
        String name = getVariableName();
        program.setUniform(name, value);
    }

    /**
     * Update the uniform's value.
     */
    @Override
    void updateValue() {
        // do nothing
    }
}
