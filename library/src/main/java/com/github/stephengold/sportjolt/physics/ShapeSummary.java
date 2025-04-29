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

import com.github.stephengold.joltjni.CompoundShape;
import com.github.stephengold.joltjni.DecoratedShape;
import com.github.stephengold.joltjni.ShapeRefC;
import com.github.stephengold.joltjni.readonly.ConstShape;
import java.util.Objects;

/**
 * Summarize inputs used to generate a Mesh for a Shape. Note: immutable.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ShapeSummary {
    // *************************************************************************
    // fields

    /**
     * summary of children (for a decorated or compound shape) or {@code null}
     * if not a decorated or compound shape
     */
    final private ChildSummaryList childSummaryList;
    /**
     * revision count of the shape
     */
    final private long revisionCount;
    /**
     * reference to the shape (prevents reuse of the virtual address)
     */
    final private ShapeRefC shapeRef;
    /**
     * strategy for mesh generation
     */
    final private MeshingStrategy meshingStrategy;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new summary.
     *
     * @param shape the shape to summarize (not null, unaffected)
     * @param strategy how to generate meshes (not null)
     */
    ShapeSummary(ConstShape shape, MeshingStrategy strategy) {
        assert strategy != null;

        this.meshingStrategy = strategy;
        this.revisionCount = shape.getRevisionCount();
        this.shapeRef = shape.toRefC();

        if (shape instanceof CompoundShape) {
            CompoundShape compoundShape = (CompoundShape) shape;
            this.childSummaryList
                    = new ChildSummaryList(compoundShape, meshingStrategy);

        } else if (shape instanceof DecoratedShape) {
            DecoratedShape decoratedShape = (DecoratedShape) shape;
            this.childSummaryList
                    = new ChildSummaryList(decoratedShape, meshingStrategy);

        } else {
            this.childSummaryList = null;
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether this summary matches the specified shape.
     *
     * @param shape the shape to compare (not null, unaffected)
     * @return {@code true} for a match, otherwise {@code false}
     */
    boolean matches(ConstShape shape) {
        long thisVa = shapeRef.targetVa();
        long argVa = shape.targetVa();
        if (thisVa != argVa) {
            return false;

        } else if (revisionCount != shape.getRevisionCount()) {
            return false;

        } else if (shape instanceof CompoundShape) {
            CompoundShape compoundShape = (CompoundShape) shape;
            int numChildren = compoundShape.getNumSubShapes();
            assert childSummaryList.countChildren() == numChildren;
            for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
                boolean matches = childSummaryList.matchesChild(
                        childIndex, compoundShape);
                if (!matches) {
                    return false;
                }
            }

        } else if (shape instanceof DecoratedShape) {
            DecoratedShape decoratedShape = (DecoratedShape) shape;
            boolean matches = childSummaryList.matchesChild(decoratedShape);
            return matches;
        }

        return true;
    }

    /**
     * Return the algorithm to generate meshes.
     *
     * @return the pre-existing object
     */
    MeshingStrategy meshingStrategy() {
        return meshingStrategy;
    }

    /**
     * Return the address of the native {@code Shape}.
     *
     * @return the virtual address (not zero
     */
    long shapeVa() {
        long result = shapeRef.targetVa();
        return result;
    }
    // *************************************************************************
    // Object methods

    /**
     * Test for equivalence with another Object.
     *
     * @param otherObject the object to compare to (may be null, unaffected)
     * @return true if the objects are equivalent, otherwise false
     */
    @Override
    public boolean equals(Object otherObject) {
        boolean result;
        if (otherObject == this) {
            result = true;

        } else if (otherObject != null
                && otherObject.getClass() == getClass()) {
            ShapeSummary otherSummary = (ShapeSummary) otherObject;
            result = (shapeRef.targetVa() == otherSummary.shapeVa())
                    && meshingStrategy.equals(otherSummary.meshingStrategy())
                    && (revisionCount == otherSummary.revisionCount);
            if (result && childSummaryList != null) {
                result = childSummaryList.equals(otherSummary.childSummaryList);
            }

        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this summary.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        long shapeVa = shapeRef.targetVa();
        int hash = Objects.hash(
                childSummaryList, revisionCount, shapeVa, meshingStrategy);
        return hash;
    }
}
