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

import com.github.stephengold.joltjni.Jolt;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

/**
 * Wrapper class for the index buffer of a mesh.
 *
 * @author Stephen Gold sgold@sonic.net
 */
public class BaseIndexBuffer {
    // *************************************************************************
    // fields

    /**
     * true for mutable, or false if immutable
     */
    private boolean mutable = true;
    /**
     * interface to the buffer data, regardless of element type
     */
    final private Buffer buffer;
    /**
     * buffer data if all indices fit in a byte, otherwise null
     */
    final private ByteBuffer bytes;
    /**
     * highest vertex index that may be put in the buffer
     */
    final private int lastVertexIndex;
    /**
     * buffer data if not all indices fit in a short, otherwise null
     */
    final private IntBuffer ints;
    /**
     * buffer data if all indices fit in a short but not a byte, otherwise null
     */
    final private ShortBuffer shorts;
    // *************************************************************************
    // constructors

    /**
     * Instantiate an IndexBuffer by wrapping the specified data buffer.
     *
     * @param dataBuffer the data buffer to wrap (a ByteBuffer or ShortBuffer or
     * IntBuffer, alias created)
     */
    protected BaseIndexBuffer(Buffer dataBuffer) {
        this.buffer = dataBuffer;

        if (dataBuffer instanceof ByteBuffer) {
            this.lastVertexIndex = (1 << 8) - 1;
            this.bytes = (ByteBuffer) dataBuffer;
            this.ints = null;
            this.shorts = null;

        } else if (dataBuffer instanceof ShortBuffer) {
            this.lastVertexIndex = (1 << 16) - 1;
            this.bytes = null;
            this.ints = null;
            this.shorts = (ShortBuffer) dataBuffer;

        } else if (dataBuffer instanceof IntBuffer) {
            this.lastVertexIndex = Integer.MAX_VALUE;
            this.bytes = null;
            this.ints = (IntBuffer) dataBuffer;
            this.shorts = null;

        } else {
            Class<? extends Buffer> clazz = dataBuffer.getClass();
            throw new IllegalArgumentException(clazz.getSimpleName());
        }
    }

    /**
     * Instantiate an IndexBuffer with a new direct, writable data buffer.
     *
     * @param maxVertices one more than the highest index value (&ge;1)
     * @param capacity number of indices (&ge;0)
     */
    protected BaseIndexBuffer(int maxVertices, int capacity) {
        this.lastVertexIndex = maxVertices - 1;

        if (lastVertexIndex < (1 << 8)) {
            this.bytes = Jolt.newDirectByteBuffer(capacity);
            this.ints = null;
            this.shorts = null;
            this.buffer = bytes;

        } else if (lastVertexIndex < (1 << 16)) {
            this.bytes = null;
            this.ints = null;
            this.shorts = Jolt.newDirectShortBuffer(capacity);
            this.buffer = shorts;

        } else {
            this.bytes = null;
            this.ints = Jolt.newDirectIntBuffer(capacity);
            this.shorts = null;
            this.buffer = ints;
        }
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Copy all the indices to a new direct buffer. The new buffer's position
     * will be 0, its limit will be its capacity, its mark will be undefined.
     *
     * @return a new direct buffer
     */
    public Buffer copyBuffer() {
        if (bytes != null) {
            int numBytes = bytes.capacity();
            ByteBuffer result = Jolt.newDirectByteBuffer(numBytes);
            for (int position = 0; position < numBytes; ++position) {
                byte index = bytes.get(position);
                result.put(position, index);
            }

            return result;

        } else if (shorts != null) {
            int numShorts = shorts.capacity();
            ShortBuffer result = Jolt.newDirectShortBuffer(numShorts);
            for (int position = 0; position < numShorts; ++position) {
                short index = shorts.get(position);
                result.put(position, index);
            }

            return result;
        }

        int numInts = ints.capacity();
        IntBuffer result = Jolt.newDirectIntBuffer(numInts);
        for (int position = 0; position < numInts; ++position) {
            int index = ints.get(position);
            result.put(position, index);
        }

        return result;
    }

    /**
     * Create a mutable IndexBuffer with a new direct data buffer.
     *
     * @param maxVertices one more than the highest index value (&ge;1)
     * @param capacity number of indices (&ge;0)
     * @return a new buffer
     */
    public static BaseIndexBuffer createIndexBuffer(
            int maxVertices, int capacity) {
        Validate.nonNegative(capacity, "capacity");
        Validate.positive(maxVertices, "max number of vertices");

        BaseIndexBuffer result = new BaseIndexBuffer(maxVertices, capacity);
        return result;
    }

    /**
     * Read an index from the specified buffer position. Does not alter the
     * buffer's read/write position.
     *
     * @param position the position from which to read (&ge;0, &lt;limit)
     * @return the index that was read (&ge;0, &lt;numVertices)
     */
    public int get(int position) {
        int result;
        if (bytes != null) {
            result = bytes.get(position) & 0xFF;
        } else if (shorts != null) {
            result = shorts.get(position) & 0xFFFF;
        } else {
            result = ints.get(position);
        }

        assert result >= 0 && result <= lastVertexIndex : result;
        return result;
    }

    /**
     * Test whether the buffer is direct.
     *
     * @return {@code true} if direct, otherwise false
     */
    public boolean isDirect() {
        boolean result = buffer.isDirect();
        return result;
    }

    /**
     * Alter the limit. If the read/write position is past the new limit then it
     * is set to the new limit.
     *
     * @param newLimit the desired limit position (&ge;0, &le;capacity);
     * @return the (modified) current instance (for chaining)
     */
    public BaseIndexBuffer limit(int newLimit) {
        verifyMutable();
        buffer.limit(newLimit);
        return this;
    }

    /**
     * Write the specified index at the specified buffer position. Does not
     * alter the buffer's read/write position.
     *
     * @param position the position to write to (&ge;0, &lt;limit)
     * @param index the index to be written (&ge;0, &lt;numVertices)
     * @return the (modified) current instance (for chaining)
     */
    public BaseIndexBuffer put(int position, int index) {
        verifyMutable();
        Validate.inRange(index, "index", 0, lastVertexIndex);

        if (bytes != null) {
            bytes.put(position, (byte) index);
        } else if (shorts != null) {
            shorts.put(position, (short) index);
        } else {
            ints.put(position, index);
        }

        return this;
    }

    /**
     * Return the buffer's limit.
     *
     * @return the element count (&ge;0)
     */
    public int size() {
        int result = buffer.limit();
        return result;
    }

    /**
     * Create a mutable IndexBuffer by wrapping the specified data buffer.
     *
     * @param dataBuffer the data buffer to wrap (not null, a ByteBuffer or
     * ShortBuffer or IntBuffer, alias created)
     * @return a new buffer
     */
    public static BaseIndexBuffer wrapIndexBuffer(Buffer dataBuffer) {
        Validate.nonNull(dataBuffer, "data buffer");

        BaseIndexBuffer result = new BaseIndexBuffer(dataBuffer);
        return result;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Read an index from the current read/write position, then increment the
     * position.
     *
     * @return the index that was read (&ge;0, &lt;numVertices)
     */
    protected int get() {
        int result;
        if (bytes != null) {
            result = bytes.get() & 0xFF;
        } else if (shorts != null) {
            result = shorts.get() & 0xFFFF;
        } else {
            result = ints.get();
        }

        assert result >= 0 && result <= lastVertexIndex : result;
        return result;
    }

    /**
     * Access the buffer data.
     *
     * @return the pre-existing buffer
     */
    protected Buffer getBuffer() {
        assert buffer != null;
        return buffer;
    }

    /**
     * Make the buffer immutable.
     *
     * @return the (modified) current instance (for chaining)
     */
    protected BaseIndexBuffer makeImmutable() {
        this.mutable = false;
        return this;
    }

    /**
     * Write the specified index at the current read/write position, then
     * increment the position.
     *
     * @param index the index to be written (&ge;0, &lt;numVertices)
     * @return the (modified) current instance (for chaining)
     */
    protected BaseIndexBuffer put(int index) {
        verifyMutable();
        Validate.inRange(index, "index", 0, lastVertexIndex);

        if (bytes != null) {
            bytes.put((byte) index);
        } else if (shorts != null) {
            shorts.put((short) index);
        } else {
            ints.put(index);
        }

        return this;
    }

    /**
     * Verify that the buffer is still mutable.
     */
    protected void verifyMutable() {
        if (!mutable) {
            throw new IllegalStateException(
                    "The index buffer is no longer mutable.");
        }
    }
}
