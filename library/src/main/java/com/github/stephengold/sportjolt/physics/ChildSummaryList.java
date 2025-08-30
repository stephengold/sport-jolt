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
import com.github.stephengold.joltjni.readonly.ConstShape;
import com.github.stephengold.joltjni.readonly.ConstSubShape;
import java.util.Arrays;

/**
 * An immutable list of child-shape summaries.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class ChildSummaryList {
    // *************************************************************************
    // fields

    /**
     * array of child summaries (all non-null)
     */
    final private ShapeSummary[] summaries;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new list for the specified compound shape.
     *
     * @param compoundShape the compound shape (not null, unaffected)
     * @param strategy how to generate meshes (not null)
     */
    ChildSummaryList(CompoundShape compoundShape, MeshingStrategy strategy) {
        assert strategy != null;

        int numChildren = compoundShape.getNumSubShapes();
        this.summaries = new ShapeSummary[numChildren];

        for (int childIndex = 0; childIndex < numChildren; ++childIndex) {
            ConstSubShape subShape = compoundShape.getSubShape(childIndex);
            ConstShape childShape = subShape.getShape();
            summaries[childIndex] = new ShapeSummary(childShape, strategy);
        }
    }

    /**
     * Instantiate a new list for the specified decorated shape.
     *
     * @param decoratedShape the decorated shape (not null, unaffected)
     * @param strategy how to generate meshes (not null)
     */
    ChildSummaryList(DecoratedShape decoratedShape, MeshingStrategy strategy) {
        assert strategy != null;

        this.summaries = new ShapeSummary[1];
        ConstShape childShape = decoratedShape.getInnerShape();
        summaries[0] = new ShapeSummary(childShape, strategy);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Count how many children are in the summarized list.
     *
     * @return the count (&ge;0)
     */
    int countChildren() {
        int result = summaries.length;
        return result;
    }

    /**
     * Test whether the indexed summary matches the specified child shape.
     *
     * @param childIndex (&ge;0)
     * @param compound the compound shape to compare (not null, unaffected)
     * @return {@code true} for a match, otherwise {@code false}
     */
    boolean matchesChild(int childIndex, CompoundShape compound) {
        ConstSubShape subShape = compound.getSubShape(childIndex);
        ConstShape childShape = subShape.getShape();
        boolean result = summaries[childIndex].matches(childShape);
        return result;
    }

    /**
     * Test whether the summary matches the specified decorated shape.
     *
     * @param decorated the decorated shape to compare (not null, unaffected)
     * @return {@code true} for a match, otherwise {@code false}
     */
    boolean matchesChild(DecoratedShape decorated) {
        ConstShape childShape = decorated.getInnerShape();
        boolean result = summaries[0].matches(childShape);
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
            ChildSummaryList other = (ChildSummaryList) otherObject;
            result = Arrays.equals(summaries, other.summaries);

        } else {
            result = false;
        }

        return result;
    }

    /**
     * Generate the hash code for this list.
     *
     * @return a 32-bit value for use in hashing
     */
    @Override
    public int hashCode() {
        int hash = Arrays.hashCode(summaries);
        return hash;
    }
}
