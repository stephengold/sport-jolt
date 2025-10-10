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

import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.operator.Op;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RMat44Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.joml.Vector2f;
import org.joml.Vector2fc;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4fc;
import org.lwjgl.opengl.GL11C;
import org.lwjgl.opengl.GL30C;

/**
 * Encapsulate a vertex array object (VAO), to which vertex buffer objects
 * (VBOs) are attached. The VAO is created lazily, the first time
 * {@link #enableAttributes(ShaderProgram)} is invoked.
 */
public class Mesh {
    // *************************************************************************
    // constants

    /**
     * number of axes in a 3-D vector
     */
    final protected static int numAxes = 3;
    /**
     * number of vertices per edge (line)
     */
    final public static int vpe = 2;
    /**
     * number of vertices per triangle
     */
    final public static int vpt = 3;
    // *************************************************************************
    // fields

    /**
     * true for a mutable mesh, or false if immutable
     */
    private boolean mutable = true;
    /**
     * vertex indices, or null if none
     */
    private IndexBuffer indexBuffer;
    /**
     * number of vertices (based on buffer sizes, unmodified by indexing)
     */
    final private int vertexCount;
    /**
     * OpenGL name of the VAO (for binding or deleting) or {@code null} if it
     * hasn't been generated yet
     */
    private Integer vaoName;
    /**
     * how vertices are organized into primitives (not null)
     */
    private Topology topology;
    /**
     * vertex colors (3 floats per vertex) or null if not present
     */
    private VertexBuffer colorBuffer;
    /**
     * vertex normals (3 floats per vertex) or null if not present
     */
    private VertexBuffer normalBuffer;
    /**
     * vertex positions (3 floats per vertex)
     */
    private VertexBuffer positionBuffer;
    /**
     * vertex texture coordinates (2 floats per vertex) or null if not present
     */
    private VertexBuffer texCoordsBuffer;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a mutable mesh from vertices and optional indices.
     *
     * @param topology the desired primitive topology (not null)
     * @param indices the vertex indices to use (unaffected) or null if none
     * @param vertices the vertex data to use (not null, unaffected)
     */
    public Mesh(Topology topology, Collection<Integer> indices,
            List<Vertex> vertices) {
        Validate.nonNull(topology, "topology");

        this.topology = topology;
        this.vertexCount = vertices.size();
        if (indices != null) {
            this.indexBuffer = IndexBuffer.newInstance(indices);
        }

        // position buffer:
        this.positionBuffer = VertexBuffer.newPosition(vertices);
        Vertex representativeVertex = vertices.get(0);

        // color buffer:
        boolean hasColor = representativeVertex.hasColor();
        if (hasColor) {
            this.colorBuffer = VertexBuffer.newColor(vertices);
        }

        // normal buffer:
        boolean hasNormal = representativeVertex.hasNormal();
        if (hasNormal) {
            this.normalBuffer = VertexBuffer.newNormal(vertices);
        }

        // texture-coordinates buffer:
        boolean hasTexCoords = representativeVertex.hasTexCoords();
        if (hasTexCoords) {
            this.texCoordsBuffer = VertexBuffer.newTexCoords(vertices);
        }

        assert mutable;
    }

    /**
     * Instantiate a mutable mesh with the specified topology and vertex
     * positions, but no indices, colors, normals, or texture coordinates.
     *
     * @param topology the desired topology (not null)
     * @param positionsArray vertex positions (not null, not empty, length a
     * multiple of 3, unaffected)
     */
    public Mesh(Topology topology, float... positionsArray) {
        this(topology, positionsArray.length / numAxes);
        Validate.require(
                positionsArray.length % numAxes == 0, "length a multiple of 3");

        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, numAxes, positionsArray);
    }

    /**
     * Instantiate a mutable mesh with the specified topology and vertex
     * positions, but no indices, colors, normals, or texture coordinates.
     *
     * @param topology the desired primitive topology (not null)
     * @param positionsBuffer the desired vertex positions (not null, capacity a
     * multiple of 3)
     */
    protected Mesh(Topology topology, FloatBuffer positionsBuffer) {
        this(topology, positionsBuffer.capacity() / numAxes);
        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, numAxes, positionsBuffer);
    }

    /**
     * Instantiate an incomplete mutable mesh with the specified topology and
     * number of vertices, but no indices, positions, colors, normals, or
     * texture coordinates.
     *
     * @param topology the desired primitive topology (not null)
     * @param vertexCount the expected number of vertices (&ge;0)
     */
    protected Mesh(Topology topology, int vertexCount) {
        Validate.nonNull(topology, "topology");
        Validate.nonNegative(vertexCount, "vertex count");

        this.topology = topology;
        this.vertexCount = vertexCount;
    }

    /**
     * Instantiate a mutable mesh with the specified topology and vertex
     * positions, but no indices, colors, normals, or texture coordinates.
     *
     * @param topology an enum value (not null)
     * @param positionsArray vertex positions (in mesh coordinates, not null,
     * not empty)
     */
    public Mesh(Topology topology, Vec3Arg... positionsArray) {
        this(topology, positionsArray.length);
        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, positionsArray);
    }

    /**
     * Instantiate a mutable mesh with the specified topology and vertex
     * positions, but no indices, colors, normals, or texture coordinates.
     *
     * @param topology an enum value (not null)
     * @param positionsArray vertex positions (in mesh coordinates, not null,
     * not empty)
     */
    public Mesh(Topology topology, Vector3fc... positionsArray) {
        this(topology, positionsArray.length);
        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, positionsArray);
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Delete the VAO and all its VBOs.
     */
    public void cleanUp() {
        if (vaoName == null) {
            return;
        }

        GL30C.glBindVertexArray(vaoName);
        Utils.checkForOglError();

        if (indexBuffer != null) {
            indexBuffer.cleanUp();
        }
        if (positionBuffer != null) {
            positionBuffer.cleanUp();
        }
        if (colorBuffer != null) {
            colorBuffer.cleanUp();
        }
        if (normalBuffer != null) {
            normalBuffer.cleanUp();
        }
        if (texCoordsBuffer != null) {
            texCoordsBuffer.cleanUp();
        }

        GL30C.glDeleteVertexArrays(vaoName);
        Utils.checkForOglError();
    }

    /**
     * Copy a single vertex from the mesh, which is unaffected.
     *
     * @param vertexIndex the vertex index (&ge;0, &lt;vertexCount)
     * @return a new vertex
     */
    Vertex copyVertex(int vertexIndex) {
        Validate.inRange(vertexIndex, "vertex index", 0, vertexCount - 1);

        Vector3fc position = positionBuffer.get3f(vertexIndex, null);

        Vector3fc color = null;
        if (colorBuffer != null) {
            color = colorBuffer.get3f(vertexIndex, null);
        }

        Vector3fc normal = null;
        if (normalBuffer != null) {
            normal = normalBuffer.get3f(vertexIndex, null);
        }

        Vector2fc texCoords = null;
        if (texCoordsBuffer != null) {
            texCoords = texCoordsBuffer.get2f(vertexIndex, null);
        }

        Vertex result = new Vertex(position, color, normal, texCoords);
        return result;
    }

    /**
     * Count how many vertices the mesh renders, taking indexing into account,
     * but not the topology. The mesh is unaffected.
     *
     * @return the count (&ge;0)
     */
    public int countIndexedVertices() {
        int result
                = (indexBuffer == null) ? vertexCount : indexBuffer.capacity();
        return result;
    }

    /**
     * Count how many line primitives the mesh contains, taking indexing and
     * topology into account. The mesh is unaffected.
     *
     * @return the count (&ge;0)
     */
    public int countLines() {
        int result;
        int vpp = topology.vpp();
        if (vpp == 2) {
            int numIndices = countIndexedVertices();
            int numShared = topology.numShared();
            result = (numIndices - numShared) / (vpp - numShared);
            if (topology == Topology.LineLoop) {
                ++result; // complete the loop
            }
        } else {
            result = 0;
        }

        return result;
    }

    /**
     * Count how many point primitives the mesh contains, taking indexing and
     * topology into account. The mesh is unaffected.
     *
     * @return the count (&ge;0)
     */
    public int countPoints() {
        int result;
        if (topology == Topology.PointList) {
            result = countIndexedVertices();
        } else {
            result = 0;
        }

        return result;
    }

    /**
     * Count how many triangle primitives the mesh contains, taking indexing and
     * topology into account. The mesh is unaffected.
     *
     * @return the count (&ge;0)
     */
    public int countTriangles() {
        int result;
        int vpp = topology.vpp();
        if (vpp == 3) {
            int numIndices = countIndexedVertices();
            int numShared = topology.numShared();
            result = (numIndices - numShared) / (vpp - numShared);
        } else {
            result = 0;
        }

        return result;
    }

    /**
     * Count how many vertices the mesh contains, based on buffer capacities,
     * unmodified by primitive topology and indexing. The mesh is unaffected.
     *
     * @return the count (&ge;0)
     */
    public int countVertices() {
        return vertexCount;
    }

    /**
     * Remove the colors, if any.
     */
    public void dropColors() {
        verifyMutable();
        this.colorBuffer = null;
    }

    /**
     * Remove the normals, if any.
     */
    public void dropNormals() {
        verifyMutable();
        this.normalBuffer = null;
    }

    /**
     * Remove the texture coordinates, if any.
     */
    public void dropTexCoords() {
        verifyMutable();
        this.texCoordsBuffer = null;
    }

    /**
     * Generate normals on a triangle-by-triangle basis for a non-indexed,
     * TriangleList mesh. Any pre-existing normals are discarded.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Mesh generateFacetNormals() {
        verifyMutable();
        if (topology != Topology.TriangleList) {
            throw new IllegalStateException("topology = " + topology);
        }
        if (indexBuffer != null) {
            throw new IllegalStateException("must be non-indexed");
        }
        int numTriangles = countTriangles();
        assert vertexCount == vpt * numTriangles;

        Vec3 posA = new Vec3();
        Vec3 posB = new Vec3();
        Vec3 posC = new Vec3();

        createNormals();
        for (int triIndex = 0; triIndex < numTriangles; ++triIndex) {
            int trianglePosition = triIndex * vpt * numAxes;
            positionBuffer.get(trianglePosition, posA);
            positionBuffer.get(trianglePosition + numAxes, posB);
            positionBuffer.get(trianglePosition + 2 * numAxes, posC);

            Vec3Arg ab = Op.minus(posB, posA);
            Vec3Arg ac = Op.minus(posC, posA);
            Vec3 normal = ab.cross(ac);
            normal.normalizeInPlace();

            for (int j = 0; j < vpt; ++j) {
                normalBuffer.put(normal);
            }
        }
        normalBuffer.flip();
        assert normalBuffer.limit() == normalBuffer.capacity();

        return this;
    }

    /**
     * Generate normals using the specified strategy. Any pre-existing normals
     * are discarded.
     *
     * @param option how to generate the normals (not null)
     * @return the (modified) current instance (for chaining)
     */
    public Mesh generateNormals(NormalsOption option) {
        switch (option) {
            case Facet:
                generateFacetNormals();
                break;

            case None:
                this.normalBuffer = null;
                break;

            case Smooth:
                generateFacetNormals();
                smoothNormals();
                break;

            case Sphere:
                generateSphereNormals();
                break;

            default:
                throw new IllegalArgumentException("option = " + option);
        }

        return this;
    }

    /**
     * Generate normals on a vertex-by-vertex basis for an outward-facing
     * sphere. Any pre-existing normals are discarded.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Mesh generateSphereNormals() {
        verifyMutable();
        createNormals();

        Vector3f tmpVector = new Vector3f();
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            positionBuffer.get3f(vertexIndex, tmpVector);
            tmpVector.normalize();
            normalBuffer.put3f(vertexIndex, tmpVector);
        }

        return this;
    }

    /**
     * Generate texture coordinates using the specified strategy and
     * coefficients. Any pre-existing texture coordinates are discarded.
     *
     * @param option how to generate the texture coordinates (not null)
     * @param uCoefficients the coefficients for generating the first (U)
     * texture coordinate (not null)
     * @param vCoefficients the coefficients for generating the 2nd (V) texture
     * coordinate (not null)
     * @return the (modified) current instance (for chaining)
     */
    public Mesh generateUvs(UvsOption option, Vector4fc uCoefficients,
            Vector4fc vCoefficients) {
        verifyMutable();
        if (option == UvsOption.None) {
            texCoordsBuffer = null;
            return this;
        }
        createUvs();

        Vec3 tmpVector = new Vec3();
        for (int vertIndex = 0; vertIndex < vertexCount; ++vertIndex) {
            int inPosition = vertIndex * numAxes;
            positionBuffer.get(inPosition, tmpVector);
            switch (option) {
                case Linear:
                    break;

                case Spherical:
                    Utils.toSpherical(tmpVector);
                    tmpVector.scaleInPlace(
                            1f, Constants.invPi, Constants.invPi);
                    break;

                default:
                    throw new IllegalArgumentException("option = " + option);
            }

            float u = uCoefficients.dot(
                    tmpVector.getX(), tmpVector.getY(), tmpVector.getZ(), 1f);
            float v = vCoefficients.dot(
                    tmpVector.getX(), tmpVector.getY(), tmpVector.getZ(), 1f);
            texCoordsBuffer.put(u).put(v);
        }
        texCoordsBuffer.flip();
        assert texCoordsBuffer.limit() == texCoordsBuffer.capacity();

        return this;
    }

    /**
     * Access the positions VertexBuffer.
     *
     * @return the pre-existing buffer (not null)
     */
    public VertexBuffer getPositions() {
        return positionBuffer;
    }

    /**
     * Test whether the mesh is indexed. It is unaffected.
     *
     * @return true if indexed, otherwise false
     */
    boolean isIndexed() {
        if (indexBuffer == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Make the mesh immutable.
     *
     * @return the (modified) current instance (for chaining)
     */
    public Mesh makeImmutable() {
        this.mutable = false;
        positionBuffer.makeImmutable();
        if (indexBuffer != null) {
            indexBuffer.makeImmutable();
        }
        if (colorBuffer != null) {
            colorBuffer.makeImmutable();
        }
        if (normalBuffer != null) {
            normalBuffer.makeImmutable();
        }
        if (texCoordsBuffer != null) {
            texCoordsBuffer.makeImmutable();
        }

        return this;
    }

    /**
     * Render using the specified ShaderProgram.
     *
     * @param program the program to use (not null)
     */
    void renderUsing(ShaderProgram program) {
        program.use();
        enableAttributes(program);

        GL30C.glBindVertexArray(vaoName);
        Utils.checkForOglError();

        if (indexBuffer == null) {
            int code = topology.code();
            int startVertex = 0;
            GL11C.glDrawArrays(code, startVertex, vertexCount);
            Utils.checkForOglError();

        } else {
            indexBuffer.drawElements(topology);
        }
    }

    /**
     * Create a mutable mesh by de-duplicating a list of vertices.
     *
     * @param topology the desired primitive topology (not null)
     * @param vertices the vertex data to use (not null, unaffected)
     * @return a new mesh
     */
    public static Mesh newInstance(Topology topology, List<Vertex> vertices) {
        Validate.nonNull(topology, "topology");

        int count = vertices.size();
        List<Integer> tempIndices = new ArrayList<>(count);
        List<Vertex> tempVertices = new ArrayList<>(count);
        Map<Vertex, Integer> tempMap = new HashMap<>(count);

        for (Vertex vertex : vertices) {
            Integer index = tempMap.get(vertex);
            if (index == null) { // assign a new index to the vertex
                int nextIndex = tempVertices.size();
                tempIndices.add(nextIndex);
                tempVertices.add(vertex);
                tempMap.put(vertex, nextIndex);
            } else { // reuse an index that's already been assigned
                tempIndices.add(index);
            }
        }

        Mesh result = new Mesh(topology, tempIndices, tempVertices);
        return result;
    }

    /**
     * Apply the specified rotation to all vertices.
     *
     * @param xAngle the X rotation angle (in radians)
     * @param yAngle the Y rotation angle (in radians)
     * @param zAngle the Z rotation angle (in radians)
     * @return the (modified) current instance (for chaining)
     */
    public Mesh rotate(float xAngle, float yAngle, float zAngle) {
        if (xAngle == 0f && yAngle == 0f && zAngle == 0f) {
            return this;
        }
        verifyMutable();

        Quat quaternion = Quat.sEulerAngles(xAngle, yAngle, zAngle);

        positionBuffer.rotate(quaternion);
        if (normalBuffer != null) {
            normalBuffer.rotate(quaternion);
        }

        return this;
    }

    /**
     * Apply the specified scaling to all vertices.
     *
     * @param scaleFactor the scale factor to apply
     * @return the (modified) current instance (for chaining)
     */
    public Mesh scale(float scaleFactor) {
        if (scaleFactor == 1f) {
            return this;
        }
        verifyMutable();

        positionBuffer.scale(scaleFactor);
        return this;
    }

    /**
     * Alter the primitive topology, which determines how vertices/indices are
     * organized into primitives.
     *
     * @param topology the enum value for the desired topology (not null)
     */
    public void setTopology(Topology topology) {
        verifyMutable();
        this.topology = topology;
    }

    /**
     * Return the primitive topology, which indicates how mesh vertices/indices
     * are organized into primitives. The mesh is unaffected.
     *
     * @return an enum value (not null)
     */
    public Topology topology() {
        assert topology != null;
        return topology;
    }

    /**
     * Apply the specified transform to all vertices.
     *
     * @param transform the transform to apply (not null, unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Mesh transform(RMat44Arg transform) {
        Validate.nonNull(transform, "transform");
        if (transform.isIdentity()) {
            return this;
        }
        verifyMutable();

        positionBuffer.transform(transform);

        if (normalBuffer != null) {
            QuatArg rotation = transform.getQuaternion();
            normalBuffer.rotate(rotation);
        }

        return this;
    }

    /**
     * Transform all texture coordinates using the specified coefficients. Note
     * that the Z components of the coefficients are currently unused.
     *
     * @param uCoefficients the coefficients for calculating new Us (not null,
     * unaffected)
     * @param vCoefficients the coefficients for calculating new Vs (not null,
     * unaffected)
     * @return the (modified) current instance (for chaining)
     */
    public Mesh transformUvs(Vector4fc uCoefficients, Vector4fc vCoefficients) {
        verifyMutable();
        if (texCoordsBuffer == null) {
            throw new IllegalStateException("There are no UVs in the mesh.");
        }
        Vector2f tmpVector = new Vector2f();

        for (int vIndex = 0; vIndex < vertexCount; ++vIndex) {
            texCoordsBuffer.get2f(vIndex, tmpVector);

            float newU = uCoefficients.w()
                    + uCoefficients.x() * tmpVector.x
                    + uCoefficients.y() * tmpVector.y;
            float newV = vCoefficients.w()
                    + vCoefficients.x() * tmpVector.x
                    + vCoefficients.y() * tmpVector.y;
            tmpVector.set(newU, newV);

            texCoordsBuffer.put2f(vIndex, tmpVector);
        }

        return this;
    }
    // *************************************************************************
    // new protected methods

    /**
     * Create a buffer for putting vertex colors.
     *
     * @return a new buffer with a capacity of 3 * vertexCount floats
     */
    protected VertexBuffer createColors() {
        verifyMutable();
        this.colorBuffer = VertexBuffer.newInstance(
                ShaderProgram.colorAttribName, 3, vertexCount);
        return colorBuffer;
    }

    /**
     * Create a buffer for putting vertex indices.
     *
     * @param capacity the desired capacity (in indices, &ge;0)
     * @return a new IndexBuffer with the specified capacity
     */
    protected IndexBuffer createIndices(int capacity) {
        verifyMutable();
        this.indexBuffer = IndexBuffer.newInstance(vertexCount, capacity);
        return indexBuffer;
    }

    /**
     * Replace any existing index buffer with a new one containing the specified
     * indices.
     *
     * @param indices the desired vertex indices (not null, unaffected)
     * @return a new IndexBuffer with the specified capacity
     */
    protected IndexBuffer createIndices(List<Integer> indices) {
        verifyMutable();
        this.indexBuffer = IndexBuffer.newInstance(indices);
        return indexBuffer;
    }

    /**
     * Create a buffer for putting vertex normals.
     *
     * @return a new buffer with a capacity of 3 * vertexCount floats
     */
    protected VertexBuffer createNormals() {
        verifyMutable();
        if (countTriangles() == 0) {
            throw new IllegalStateException(
                    "The mesh doesn't contain any triangles.");
        }

        this.normalBuffer = VertexBuffer.newInstance(
                ShaderProgram.normalAttribName, numAxes, vertexCount);
        return normalBuffer;
    }

    /**
     * Create a buffer for putting vertex positions.
     *
     * @return a new buffer with a capacity of 3 * vertexCount floats
     */
    protected VertexBuffer createPositions() {
        verifyMutable();
        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, numAxes, vertexCount);
        return positionBuffer;
    }

    /**
     * Create a buffer for putting vertex texture coordinates.
     *
     * @return a new buffer with a capacity of 2 * vertexCount floats
     */
    protected VertexBuffer createUvs() {
        verifyMutable();
        this.texCoordsBuffer = VertexBuffer.newInstance(
                ShaderProgram.uvAttribName, 2, vertexCount);
        return texCoordsBuffer;
    }

    /**
     * Assign new vertex indices.
     *
     * @param indexArray the vertex indices to use (not null, unaffected)
     */
    protected void setIndices(int... indexArray) {
        verifyMutable();
        this.indexBuffer = IndexBuffer.newInstance(indexArray);
    }

    /**
     * Assign new colors to the vertices.
     *
     * @param colorArray the desired vertex colors (not null,
     * length=3*vertexCount, unaffected)
     */
    protected void setColors(float... colorArray) {
        int numFloats = colorArray.length;
        Validate.require(numFloats == vertexCount * 3, "correct length");
        verifyMutable();

        this.colorBuffer = VertexBuffer.newInstance(
                ShaderProgram.colorAttribName, 3, colorArray);
    }

    /**
     * Assign new normals to the vertices.
     *
     * @param normalArray the desired vertex normals (not null,
     * length=3*vertexCount, unaffected)
     */
    protected void setNormals(float... normalArray) {
        int numFloats = normalArray.length;
        Validate.require(numFloats == vertexCount * numAxes, "correct length");
        verifyMutable();

        this.normalBuffer = VertexBuffer.newInstance(
                ShaderProgram.normalAttribName, numAxes, normalArray);
    }

    /**
     * Assign new positions to the vertices.
     *
     * @param positionArray the desired vertex positions (not null,
     * length=3*vertexCount, unaffected)
     */
    protected void setPositions(float... positionArray) {
        int numFloats = positionArray.length;
        Validate.require(numFloats == vertexCount * numAxes, "correct length");
        verifyMutable();

        this.positionBuffer = VertexBuffer.newInstance(
                ShaderProgram.positionAttribName, numAxes, positionArray);
    }

    /**
     * Assign new texture coordinates to the vertices.
     *
     * @param uvArray the desired vertex texture coordinates (not null,
     * length=2*vertexCount, unaffected)
     */
    protected void setUvs(float... uvArray) {
        int numFloats = uvArray.length;
        Validate.require(numFloats == 2 * vertexCount, "correct length");
        verifyMutable();

        this.texCoordsBuffer = VertexBuffer.newInstance(
                ShaderProgram.uvAttribName, 2, uvArray);
    }

    /**
     * Access the index buffer.
     *
     * @return the pre-existing instance (not null)
     */
    public IndexBuffer getIndexBuffer() {
        assert indexBuffer != null;
        return indexBuffer;
    }

    /**
     * Access the normals data buffer.
     *
     * @return the pre-existing direct buffer (not null)
     */
    public FloatBuffer getNormalsData() {
        return normalBuffer.getData();
    }

    /**
     * Access the positions data buffer.
     *
     * @return the pre-existing direct buffer (not null)
     */
    public FloatBuffer getPositionsData() {
        return positionBuffer.getData();
    }

    /**
     * Test whether the topology is LineList. Indexing is ignored.
     *
     * @return true if pure lines, otherwise false
     */
    public boolean isPureLines() {
        boolean result = (topology == Topology.LineList);
        return result;
    }

    /**
     * Test whether the topology is TriangleList. Indexing is ignored.
     *
     * @return true if pure triangles, otherwise false
     */
    public boolean isPureTriangles() {
        boolean result = (topology == Topology.TriangleList);
        return result;
    }

    /**
     * Indicate that the normals data buffer is dirty.
     */
    public void setNormalsModified() {
        normalBuffer.setModified();
    }

    /**
     * Indicate that the positions data buffer is dirty.
     */
    public void setPositionsModified() {
        positionBuffer.setModified();
    }
    // *************************************************************************
    // Object methods

    /**
     * Represent the mesh as a text string.
     *
     * @return a descriptive string of text (not null)
     */
    @Override
    public String toString() {
        // Determine how many vertices to describe:
        int numToDescribe = countIndexedVertices();
        if (numToDescribe > 12) {
            numToDescribe = 12;
        }

        StringBuilder result = new StringBuilder(80 * (1 + numToDescribe));
        if (indexBuffer == null) {
            result.append("non");
        } else {
            int indexType = indexBuffer.indexType();
            String elementString = Utils.describeCode(indexType);
            result.append(elementString);
        }
        result.append("-indexed ");
        result.append(topology);
        result.append(" mesh (");
        result.append(vertexCount);
        if (vertexCount == 1) {
            result.append(" vertex");
        } else {
            result.append(" vertices");
        }

        if (indexBuffer != null) {
            result.append(", ");
            int numIndices = indexBuffer.capacity();
            result.append(numIndices);
            if (numIndices == 1) {
                result.append(" index");
            } else {
                result.append(" indices");
            }
        }

        int numTriangles = countTriangles();
        if (numTriangles > 0) {
            result.append(", ");
            result.append(numTriangles);
            result.append(" triangle");
            if (numTriangles != 1) {
                result.append("s");
            }
        }

        int numLines = countLines();
        if (numLines > 0) {
            result.append(", ");
            result.append(numLines);
            result.append(" line");
            if (numLines != 1) {
                result.append("s");
            }
        }
        result.append(")");
        String nl = System.lineSeparator();
        result.append(nl);
        /*
         * In the body of the description, vertices are grouped into primitives,
         * separated by empty lines.
         */
        int vpp = topology.vpp();
        int numShared = topology.numShared();

        for (int i = 0; i < numToDescribe; ++i) {
            if (i >= vpp && ((i - numShared) % (vpp - numShared)) == 0) {
                result.append(nl);
            }

            int vertexIndex = (indexBuffer == null) ? i : indexBuffer.get(i);
            result.append(vertexIndex);
            result.append(": ");
            Vertex v = copyVertex(vertexIndex);
            result.append(v);
            result.append(nl);
        }
        if (countIndexedVertices() > numToDescribe) {
            result.append("...");
            result.append(nl);
        }

        return result.toString();
    }
    // *************************************************************************
    // private methods

    /**
     * Prepare all vertex attributes for rendering.
     * <p>
     * If the VAO doesn't already exist, it is created.
     *
     * @param program (not null)
     */
    private void enableAttributes(ShaderProgram program) {
        if (vaoName == null) {
            this.vaoName = GL30C.glGenVertexArrays();
            Utils.checkForOglError();

            GL30C.glBindVertexArray(vaoName);
            Utils.checkForOglError();

            this.mutable = false;

        } else {
            assert !mutable;

            // Use the existing VAO.
            GL30C.glBindVertexArray(vaoName);
            Utils.checkForOglError();
        }

        positionBuffer.prepareToDraw(program);
        if (colorBuffer != null) {
            colorBuffer.prepareToDraw(program);
        }
        if (normalBuffer != null) {
            normalBuffer.prepareToDraw(program);
        }
        if (texCoordsBuffer != null) {
            texCoordsBuffer.prepareToDraw(program);
        }
    }

    /**
     * Smooth the pre-existing normals by averaging them across all uses of each
     * distinct vertex position.
     */
    private void smoothNormals() {
        verifyMutable();
        assert indexBuffer == null;
        assert normalBuffer != null;

        Map<Vec3Arg, Integer> mapPosToDpid = new HashMap<>(vertexCount);
        int numDistinctPositions = 0;
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            int start = vertexIndex * numAxes;
            Vec3 position = new Vec3();
            positionBuffer.get(start, position);
            position.standardizeInPlace();
            if (!mapPosToDpid.containsKey(position)) {
                mapPosToDpid.put(position, numDistinctPositions);
                ++numDistinctPositions;
            }
        }

        // Initialize the normal sum for each distinct position.
        Vec3[] normalSums = new Vec3[numDistinctPositions];
        for (int dpid = 0; dpid < numDistinctPositions; ++dpid) {
            normalSums[dpid] = new Vec3();
        }

        final Vec3 tmpPosition = new Vec3();
        final Vec3 tmpNormal = new Vec3();
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            int start = vertexIndex * numAxes;
            positionBuffer.get(start, tmpPosition);
            tmpPosition.standardizeInPlace();
            int dpid = mapPosToDpid.get(tmpPosition);

            normalBuffer.get(start, tmpNormal);
            Op.plusEquals(normalSums[dpid], tmpNormal);
        }

        // Re-normalize the normal sum for each distinct position.
        for (Vec3 normal : normalSums) {
            normal.normalizeInPlace();
        }

        // Write new normals to the buffer.
        for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            int start = vertexIndex * numAxes;
            positionBuffer.get(start, tmpPosition);
            tmpPosition.standardizeInPlace();
            int dpid = mapPosToDpid.get(tmpPosition);
            normalBuffer.put(start, normalSums[dpid]);
        }
    }

    /**
     * Verify that the mesh is still mutable.
     */
    private void verifyMutable() {
        if (!mutable) {
            throw new IllegalStateException("The mesh is no longer mutable.");
        }
    }
}
