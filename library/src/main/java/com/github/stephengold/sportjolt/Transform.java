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
package com.github.stephengold.sportjolt;

import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Quaternionfc;
import org.joml.Vector3dc;
import org.joml.Vector3f;
import org.joml.Vector3fc;

/**
 * A 3-D coordinate transform composed of translation, rotation, and scaling.
 * The order of application is: scale, then rotate, then translate.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class Transform {
    // *************************************************************************
    // fields

    /**
     * rotation component
     */
    final private Quaternionf rotation = new Quaternionf();
    /**
     * translation component, an offset for each local axis
     */
    final private Vector3f translation = new Vector3f();
    /**
     * scaling component, a scale factor for each local axis
     */
    final private Vector3f scaling = new Vector3f(1f);
    // *************************************************************************
    // constructors

    /**
     * Explicit no-arg constructor to avoid javadoc warnings from JDK 18+.
     */
    public Transform() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Access the rotation component.
     *
     * @return the pre-existing instance (not null)
     */
    public Quaternionf getRotation() {
        return rotation;
    }

    /**
     * Access the scaling component.
     *
     * @return the pre-existing instance (not null)
     */
    public Vector3f getScale() {
        return scaling;
    }

    /**
     * Access the translation component.
     *
     * @return the pre-existing instance (not null)
     */
    public Vector3f getTranslation() {
        return translation;
    }

    /**
     * Set the current instance to the identity transform: translation=(0,0,0)
     * scaling=(1,1,1) rotation=(0,0,0,1).
     */
    public void loadIdentity() {
        rotation.identity();
        scaling.set(1f);
        translation.zero();
    }

    /**
     * Copy the argument to the rotation component.
     *
     * @param rotation the desired rotation (not null, unaffected,
     * default=(0,0,0,1))
     */
    public void setRotation(Quaternionfc rotation) {
        this.rotation.set(rotation);
    }

    /**
     * Copy the argument to all 3 components of the scaling component.
     *
     * @param factor the desired uniform scale factor (default=1)
     */
    public void setScale(float factor) {
        scaling.set(factor);
    }

    /**
     * Copy the argument to the scaling component.
     *
     * @param factors the desired scaling (not null, unaffected,
     * default=(1,1,1))
     */
    public void setScale(Vector3fc factors) {
        scaling.set(factors);
    }

    /**
     * Copy the argument to the translation component.
     *
     * @param offsets the desired offsets (not null, unaffected,
     * default=(0,0,0))
     */
    public void setTranslation(Vector3dc offsets) {
        translation.set(offsets);
    }

    /**
     * Copy the argument to the translation component.
     *
     * @param offsets the desired offsets (not null, unaffected,
     * default=(0,0,0))
     */
    public void setTranslation(Vector3fc offsets) {
        translation.set(offsets);
    }

    /**
     * Generate a rotation matrix. TODO cache the result
     *
     * @param storeResult storage for the result (not null, modified)
     */
    public void toRotationMatrix(Matrix3f storeResult) {
        storeResult.rotation(rotation);
    }

    /**
     * Generate a complete transform matrix. TODO cache the result
     *
     * @param storeResult storage for the result (not null, modified)
     */
    public void toTransformMatrix(Matrix4f storeResult) {
        storeResult.translation(translation);
        storeResult.rotate(rotation);
        storeResult.scale(scaling);
    }
    // *************************************************************************
    // Object methods

    /**
     * Describe the geometry in a string of text.
     *
     * @return a descriptive string of text (not null, not empty)
     */
    @Override
    public String toString() {
        String result = String.format("sc%s %s tr%s",
                scaling, rotation, translation);
        return result;
    }
}
