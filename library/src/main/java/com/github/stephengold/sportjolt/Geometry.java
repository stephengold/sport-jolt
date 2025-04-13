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

import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import java.nio.FloatBuffer;
import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Quaternionfc;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;
import org.joml.Vector4fc;
import org.lwjgl.opengl.GL11C;

/**
 * A 3-D object to be rendered by Sport Jolt, including a mesh, a texture, a
 * shader program, a coordinate transform, and 2 colors.
 */
public class Geometry {
    // *************************************************************************
    // fields

    /**
     * true to enable depth test, false to disable it
     */
    private boolean depthTest = true;
    /**
     * true to enable back-face culling, false to disable it
     */
    private boolean isBackCulling = true;
    /**
     * true to enable front-face culling, false to disable it
     */
    private boolean isFrontCulling;
    /**
     * true to enable wireframe rendering, false to disable it
     */
    private boolean wireframe;
    /**
     * alpha discard threshold (for transparency)
     */
    private float alphaDiscardThreshold = 0.5f;
    /**
     * point size for sprites (in pixels)
     */
    private float pointSize = 32f;
    /**
     * draw mode and vertex data for visualization
     */
    private Mesh mesh;
    /**
     * temporary storage for a rotation matrix
     */
    final private Matrix3f rm = new Matrix3f();
    /**
     * temporary storage for a transform matrix
     */
    final private Matrix4f tm = new Matrix4f();
    /**
     * rendering program
     */
    private ShaderProgram program;
    /**
     * primary texture (typically diffuse color) or null if none
     */
    private Texture texture;
    /**
     * mesh-to-world coordinate transform
     */
    final private Transform meshToWorld = new Transform();
    /**
     * material base color (in the Linear colorspace)
     */
    final private Vector4f baseColor = new Vector4f(Constants.WHITE);
    /**
     * material specular color (in the Linear colorspace)
     */
    final private Vector4f specularColor = new Vector4f(Constants.WHITE);
    // *************************************************************************
    // constructors

    /**
     * Instantiate a geometry with the specified mesh and the default
     * ShaderProgram and make it visible.
     *
     * @param mesh the desired Mesh (not null, alias created)
     */
    public Geometry(Mesh mesh) {
        this();
        Validate.nonNull(mesh, "mesh");

        this.mesh = mesh;
        BaseApplication.makeVisible(this);
    }

    /**
     * Instantiate a geometry with no mesh and the default ShaderProgram. Don't
     * make it visible.
     */
    protected Geometry() {
        this.program = getDefaultProgram();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the alpha discard threshold.
     *
     * @return the threshold value
     */
    public float alphaDiscardThreshold() {
        return alphaDiscardThreshold;
    }

    /**
     * Return a copy of the mesh-to-world scale factors.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a vector of scale factors (either {@code storeResult} or a new
     * instance, not null)
     */
    public Vector3f copyScale(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        Vector3fc scale = meshToWorld.getScale(); // alias
        result.set(scale);

        return result;
    }

    /**
     * Return the base color in the Linear colorspace.
     *
     * @return the pre-existing object (not null)
     */
    public Vector4fc getColor() {
        return baseColor;
    }

    /**
     * Access the Mesh.
     *
     * @return the pre-existing object (not null)
     */
    public Mesh getMesh() {
        assert mesh != null;
        return mesh;
    }

    /**
     * Access the shader program.
     *
     * @return the pre-existing instance (not null)
     */
    ShaderProgram getProgram() {
        assert program != null;
        return program;
    }

    /**
     * Test whether back-face culling is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isBackCulling() {
        return isBackCulling;
    }

    /**
     * Test whether depth test is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isDepthTest() {
        return depthTest;
    }

    /**
     * Test whether front-face culling is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isFrontCulling() {
        return isFrontCulling;
    }

    /**
     * Test whether wireframe mode is enabled.
     *
     * @return true if enabled, otherwise false
     */
    public boolean isWireframe() {
        return wireframe;
    }

    /**
     * Copy the location of the mesh origin.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a location vector in worldspace (either {@code storeResult} or a
     * new vector, not null)
     */
    public Vector3f location(Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;
        Vector3fc translation = meshToWorld.getTranslation(); // alias
        result.set(translation);

        return result;
    }

    /**
     * Translate by the specified offset without changing the orientation.
     *
     * @param offset the offset (in worldspace, not null, finite, unaffected)
     */
    public void move(Vector3fc offset) {
        Validate.finite(offset, "finite offset");

        Vector3f location = meshToWorld.getTranslation(); // alias
        location.add(offset);
    }

    /**
     * Return a copy of the mesh-to-world coordinate rotation.
     *
     * @param storeResult storage for the result (modified if not null)
     * @return a unit quaternion (either {@code storeResult} or a new
     * quaternion)
     */
    public Quaternionf orientation(Quaternionf storeResult) {
        Quaternionf result
                = (storeResult == null) ? new Quaternionf() : storeResult;
        Quaternionfc rotation = meshToWorld.getRotation(); // alias
        result.set(rotation);

        return result;
    }

    /**
     * Return the point size for sprites.
     *
     * @return the size (in pixels)
     */
    public float pointSize() {
        return pointSize;
    }

    /**
     * Reset the model transform so that meshspace and worldspace are the same.
     *
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry resetModelTransform() {
        meshToWorld.loadIdentity();
        return this;
    }

    /**
     * Rotate the model by the specified angle around the specified axis,
     * without translating the mesh origin.
     * <p>
     * The rotation axis is assumed to be a unit vector.
     *
     * @param angle the rotation angle (in radians, 0&rarr;no effect)
     * @param x the X component of the rotation axis
     * @param y the Y component of the rotation axis
     * @param z the Z component of the rotation axis
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry rotate(float angle, float x, float y, float z) {
        Quaternionf rotation = meshToWorld.getRotation(); // alias
        rotation.rotateAxis(angle, x, y, z, rotation);

        return this;
    }

    /**
     * Uniformly scale the model by the specified factor.
     *
     * @param factor the scaling factor (1&rarr;no effect)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry scale(float factor) {
        Vector3f scale = meshToWorld.getScale(); // alias
        scale.mul(factor);

        return this;
    }

    /**
     * Alter the alpha discard threshold.
     *
     * @param threshold the desired threshold (default=0.5)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setAlphaDiscardThreshold(float threshold) {
        this.alphaDiscardThreshold = threshold;
        return this;
    }

    /**
     * Enable or disable back-face culling.
     *
     * @param newSetting true to enable, false to disable (default=true)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setBackCulling(boolean newSetting) {
        this.isBackCulling = newSetting;
        return this;
    }

    /**
     * Alter the base color.
     *
     * @param color the desired color (in the Linear colorspace, not null,
     * unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setColor(Vector4fc color) {
        Validate.nonNull(color, "color");
        baseColor.set(color);
        return this;
    }

    /**
     * Enable or disable depth testing.
     *
     * @param newSetting true to enable, false to disable (default=true)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setDepthTest(boolean newSetting) {
        if (newSetting != depthTest) {
            this.depthTest = newSetting;
            BaseApplication.updateDeferredQueue(this);
        }

        return this;
    }

    /**
     * Enable or disable front-face culling.
     *
     * @param newSetting true to enable, false to disable (default=false)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setFrontCulling(boolean newSetting) {
        this.isFrontCulling = newSetting;
        return this;
    }

    /**
     * Alter the location of the mesh origin.
     *
     * @param x the desired X coordinate (in worldspace, finite, default=0)
     * @param y the desired Y coordinate (in worldspace, finite, default=0)
     * @param z the desired Z coordinate (in worldspace, finite, default=0)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setLocation(float x, float y, float z) {
        Validate.finite(x, "x");
        Validate.finite(y, "y");
        Validate.finite(z, "z");

        meshToWorld.getTranslation().set(x, y, z);
        return this;
    }

    /**
     * Translate the mesh origin to the specified location.
     *
     * @param location the desired location (in worldspace, not null, finite,
     * unaffected, default=(0,0,0))
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setLocation(RVec3Arg location) {
        Validate.finite(location, "location");

        float x = location.x();
        float y = location.y();
        float z = location.z();
        meshToWorld.getTranslation().set(x, y, z);

        return this;
    }

    /**
     * Translate the mesh origin to the specified location.
     *
     * @param location the desired location (in worldspace, not null, finite,
     * unaffected, default=(0,0,0))
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setLocation(Vec3Arg location) {
        Validate.finite(location, "location");

        float x = location.getX();
        float y = location.getY();
        float z = location.getZ();
        meshToWorld.getTranslation().set(x, y, z);

        return this;
    }

    /**
     * Translate the mesh origin to the specified location.
     *
     * @param location the desired location (in worldspace, not null, finite,
     * unaffected, default=(0,0,0))
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setLocation(Vector3fc location) {
        Validate.finite(location, "location");
        meshToWorld.setTranslation(location);
        return this;
    }

    /**
     * Replace the geometry's current mesh with the specified one.
     *
     * @param mesh the desired mesh (not null, alias created)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setMesh(Mesh mesh) {
        Validate.nonNull(mesh, "mesh");
        this.mesh = mesh;
        return this;
    }

    /**
     * Alter the orientation using Tait-Bryan angles, applying the rotations in
     * x-y-z extrinsic order or z-y'-x" intrinsic order.
     *
     * @param xAngle the desired X angle (in radians, finite)
     * @param yAngle the desired Y angle (in radians, finite)
     * @param zAngle the desired Z angle (in radians, finite)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setOrientation(float xAngle, float yAngle, float zAngle) {
        Validate.finite(xAngle, "x angle");
        Validate.finite(xAngle, "y angle");
        Validate.finite(xAngle, "z angle");

        Quaternionf rotation = meshToWorld.getRotation(); // alias
        rotation.rotationZYX(zAngle, yAngle, xAngle);

        return this;
    }

    /**
     * Alter the orientation without translating the mesh origin.
     * <p>
     * The rotation axis is assumed to be a unit vector.
     *
     * @param angle the desired rotation angle (in radians, finite, default=0)
     * @param x the X component of the rotation axis (&ge;-1, &le;1)
     * @param y the Y component of the rotation axis (&ge;-1, &le;1)
     * @param z the Z component of the rotation axis (&ge;-1, &le;1)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setOrientation(float angle, float x, float y, float z) {
        Validate.finite(angle, "angle");
        Validate.inRange(x, "x", -1f, 1f);
        Validate.inRange(y, "y", -1f, 1f);
        Validate.inRange(z, "z", -1f, 1f);

        Quaternionf rotation = meshToWorld.getRotation(); // alias
        rotation.setAngleAxis(angle, x, y, z);

        return this;
    }

    /**
     * Alter the orientation without translating the mesh origin.
     *
     * @param orientation the desired orientation (not null, not zero,
     * unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setOrientation(QuatArg orientation) {
        Validate.nonZero(orientation, "orientation");

        Quaternionf rotation = meshToWorld.getRotation(); // alias
        float qx = orientation.getX();
        float qy = orientation.getY();
        float qz = orientation.getZ();
        float qw = orientation.getW();
        rotation.set(qx, qy, qz, qw);

        return this;
    }

    /**
     * Alter the orientation without translating the mesh origin.
     *
     * @param orientation the desired orientation (not null, not zero,
     * unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setOrientation(Quaternionfc orientation) {
        Validate.nonNull(orientation, "orientation");

        Quaternionf rotation = meshToWorld.getRotation(); // alias
        rotation.set(orientation);

        return this;
    }

    /**
     * Alter the point size for sprites.
     *
     * @param size the desired size (in pixels, default=32)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setPointSize(float size) {
        this.pointSize = size;
        return this;
    }

    /**
     * Replace the geometry's current shader program with the named program, or
     * if the name is null, replace it with the default program.
     *
     * @param name the name of the desired program (may be null)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setProgram(String name) {
        if (name == null) {
            this.program = getDefaultProgram();
        } else {
            this.program = BaseApplication.getProgram(name);
        }

        return this;
    }

    /**
     * Alter the mesh-to-world scale factors.
     *
     * @param scaleFactor the desired mesh-to-world scale factor for all axes
     * (finite, default=1)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setScale(float scaleFactor) {
        meshToWorld.setScale(scaleFactor);
        return this;
    }

    /**
     * Alter the mesh-to-world scale factors.
     *
     * @param scaleFactors the desired scale factor for each mesh axis (not
     * null, finite, unaffected, default=(1,1,1))
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setScale(Vec3Arg scaleFactors) {
        Validate.finite(scaleFactors, "scale factors");

        Vector3f scale = meshToWorld.getScale(); // alias
        float sx = scaleFactors.getX();
        float sy = scaleFactors.getY();
        float sz = scaleFactors.getZ();
        scale.set(sx, sy, sz);

        return this;
    }

    /**
     * Alter the mesh-to-world scale factors.
     *
     * @param scaleFactors the desired scale factor for each mesh axis (not
     * null, finite, unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setScale(Vector3fc scaleFactors) {
        Validate.finite(scaleFactors, "scale factors");

        Vector3f scale = meshToWorld.getScale(); // alias
        scale.set(scaleFactors);

        return this;
    }

    /**
     * Alter the specular color.
     *
     * @param color the desired color (in the Linear colorspace, not null,
     * unaffected)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setSpecularColor(Vector4fc color) {
        Validate.nonNull(color, "color");
        specularColor.set(color);
        return this;
    }

    /**
     * Replace the primary texture with one obtained using the specified key.
     *
     * @param textureKey a key to obtain the desired texture (not null)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setTexture(TextureKey textureKey) {
        Validate.nonNull(textureKey, "texture key");
        this.texture = BaseApplication.getTexture(textureKey);
        return this;
    }

    /**
     * Enable or disable wireframe mode.
     *
     * @param newSetting true to enable, false to disable (default=false)
     * @return the (modified) current geometry (for chaining)
     */
    public Geometry setWireframe(boolean newSetting) {
        this.wireframe = newSetting;
        return this;
    }

    /**
     * Update properties and then render this Geometry. Assumes that the
     * program's global uniforms have already been set! Meant to be overridden.
     */
    public void updateAndRender() {
        if (mesh.countIndexedVertices() == 0) {
            return;
        }

        // Ensure that the program's uniforms have been collected:
        program.use();

        // mesh-to-world transform uniforms
        if (program.hasActiveUniform(
                ShaderProgram.modelRotationMatrixUniformName)) {
            program.setModelRotationMatrix(this);
        }
        if (program.hasActiveUniform(ShaderProgram.modelMatrixUniformName)) {
            program.setModelMatrix(this);
        }

        // material uniforms
        if (program.hasActiveUniform("alphaDiscardMaterialThreshold")) {
            program.setUniform("alphaDiscardMaterialThreshold",
                    alphaDiscardThreshold);
        }
        if (program.hasActiveUniform("BaseMaterialColor")) {
            program.setUniform("BaseMaterialColor", baseColor);
        }
        if (program.hasActiveUniform("ColorMaterialTexture")) {
            texture.bind();

            int unitNumber = 0;
            program.setUniform("ColorMaterialTexture", unitNumber);
        }
        if (program.hasActiveUniform("pointMaterialSize")) {
            program.setUniform("pointMaterialSize", pointSize);
        }
        if (program.hasActiveUniform("SpecularMaterialColor")) {
            program.setUniform("SpecularMaterialColor", specularColor);
        }

        GL11C.glPolygonMode(GL11C.GL_FRONT_AND_BACK,
                wireframe ? GL11C.GL_LINE : GL11C.GL_FILL);
        Utils.checkForOglError();

        Utils.setOglCapability(GL11C.GL_DEPTH_TEST, depthTest);

        boolean cullFace = (isBackCulling || isFrontCulling);
        Utils.setOglCapability(GL11C.GL_CULL_FACE, cullFace);

        if (isBackCulling && isFrontCulling) {
            GL11C.glCullFace(GL11C.GL_FRONT_AND_BACK);
            Utils.checkForOglError();

        } else if (isBackCulling) {
            GL11C.glCullFace(GL11C.GL_BACK);
            Utils.checkForOglError();

        } else if (isFrontCulling) {
            GL11C.glCullFace(GL11C.GL_FRONT);
            Utils.checkForOglError();
        }

        mesh.renderUsing(program);
    }

    /**
     * Test whether the physics object (if any) has been removed from the
     * specified {@code PhysicsSystem}. Meant to be overridden.
     *
     * @param space the CollisionSpace to test (not null)
     * @return true if removed, otherwise false
     */
    public boolean wasRemovedFrom(PhysicsSystem space) {
        return false;
    }

    /**
     * Write the mesh-to-world 3x3 rotation matrix in column-major order to the
     * specified FloatBuffer, starting at the current buffer position. The
     * buffer position is unaffected.
     *
     * @param storeBuffer the buffer to modify (not null)
     */
    void writeRotationMatrix(FloatBuffer storeBuffer) {
        meshToWorld.toRotationMatrix(rm);

        int startPosition = storeBuffer.position();
        rm.get(storeBuffer);
        storeBuffer.position(startPosition);
    }

    /**
     * Write the mesh-to-world 4x4 transform matrix in column-major order to the
     * specified FloatBuffer, starting at the current buffer position. The
     * buffer position is unaffected.
     *
     * @param storeBuffer the buffer to modify (not null)
     */
    void writeTransformMatrix(FloatBuffer storeBuffer) {
        meshToWorld.toTransformMatrix(tm);

        int startPosition = storeBuffer.position();
        tm.get(storeBuffer);
        storeBuffer.position(startPosition);
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
        String result = String.format("adt=%g meshToWorld[%s]"
                + " baseColor[%s] isBackCulling=%s depthTest=%s"
                + " isFrontCulling=%s wireframe=%s pointSize=%g",
                alphaDiscardThreshold, meshToWorld, baseColor,
                isBackCulling, depthTest, isFrontCulling, wireframe, pointSize);
        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Return the default ShaderProgram for new geometries.
     *
     * @return a valid program (not null)
     */
    private static ShaderProgram getDefaultProgram() {
        ShaderProgram result
                = BaseApplication.getProgram("Phong/Distant/Monochrome");
        return result;
    }
}
