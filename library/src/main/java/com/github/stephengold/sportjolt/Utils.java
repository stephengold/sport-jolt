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
import com.github.stephengold.joltjni.Quat;
import com.github.stephengold.joltjni.RVec3;
import com.github.stephengold.joltjni.Vec3;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.charset.StandardCharsets;
import java.util.Scanner;
import javax.imageio.ImageIO;
import org.joml.Quaternionf;
import org.joml.Quaternionfc;
import org.joml.Vector3f;
import org.joml.Vector3fc;
import org.joml.Vector4f;
import org.lwjgl.opengl.GL11C;
import org.lwjgl.opengl.GL12C;
import org.lwjgl.opengl.GL13C;
import org.lwjgl.opengl.GL14C;
import org.lwjgl.opengl.GL15C;
import org.lwjgl.opengl.GL20C;

/**
 * Public utility methods in the Sport-Jolt Library.
 */
final public class Utils {
    // *************************************************************************
    // constants

    /**
     * {@code true} if assertions are enabled, otherwise {@code false}
     */
    final private static boolean assertions = areAssertionsEnabled();
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Utils() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Test whether assertions are enabled.
     *
     * @return true if enabled, otherwise false
     */
    public static boolean areAssertionsEnabled() {
        boolean enabled = false;
        assert enabled = true; // Note: intentional side effect.

        return enabled;
    }

    /**
     * Throw a runtime exception if assertions are enabled and OpenGL has
     * detected an error since the previous invocation of this method.
     */
    public static void checkForOglError() {
        if (assertions) {
            int errorCode = GL11C.glGetError();
            if (errorCode == GL11C.GL_OUT_OF_MEMORY) {
                throw new IllegalStateException("OpenGL out of memory");
            } else if (errorCode != GL11C.GL_NO_ERROR) {
                throw new IllegalStateException("OpenGL error " + errorCode);
            }
        }
    }

    /**
     * Convert the specified OpenGL code to text. (Not all codes are handled.)
     *
     * @param code the code to decipher
     * @return a descriptive string of text
     */
    public static String describeCode(int code) {
        switch (code) {
            case GL11C.GL_COLOR_BUFFER_BIT:
                return "COLOR_BUFFER_BIT";
            case GL11C.GL_DEPTH_BUFFER_BIT:
                return "DEPTH_BUFFER_BIT";
            case GL11C.GL_BYTE:
                return "BYTE";
            case GL11C.GL_DEPTH_TEST:
                return "DEPTH_TEST";
            case GL11C.GL_DOUBLE:
                return "DOUBLE";
            case GL11C.GL_FALSE:
                return "FALSE";
            case GL11C.GL_FLOAT:
                return "FLOAT";
            case GL11C.GL_FRONT_AND_BACK:
                return "FRONT_AND_BACK";
            case GL11C.GL_INT:
                return "INT";
            case GL11C.GL_LINEAR:
                return "LINEAR";
            case GL11C.GL_LINEAR_MIPMAP_LINEAR:
                return "LINEAR_MIPMAP_LINEAR";
            case GL11C.GL_LINEAR_MIPMAP_NEAREST:
                return "LINEAR_MIPMAP_NEAREST";
            case GL11C.GL_LINE_LOOP:
                return "LINE_LOOP";
            case GL11C.GL_LINES:
                return "LINES";
            case GL11C.GL_LINE_STRIP:
                return "LINE_STRIP";

            case GL11C.GL_NEAREST:
                return "NEAREST";
            case GL11C.GL_NEAREST_MIPMAP_LINEAR:
                return "NEAREST_MIPMAP_LINEAR";
            case GL11C.GL_NEAREST_MIPMAP_NEAREST:
                return "NEAREST_MIPMAP_NEAREST";
            case GL11C.GL_QUADS:
                return "QUADS";
            case GL11C.GL_REPEAT:
                return "REPEAT";
            case GL11C.GL_RGBA:
                return "RGBA";
            case GL11C.GL_SHORT:
                return "SHORT";
            case GL11C.GL_TRIANGLE_FAN:
                return "TRIANGLE_FAN";
            case GL11C.GL_TRIANGLES:
                return "TRIANGLES";
            case GL11C.GL_TRIANGLE_STRIP:
                return "TRIANGLE_STRIP";
            case GL11C.GL_UNSIGNED_BYTE:
                return "UNSIGNED_BYTE";
            case GL11C.GL_UNSIGNED_INT:
                return "UNSIGNED_INT";
            case GL11C.GL_UNSIGNED_SHORT:
                return "UNSIGNED_SHORT";

            case GL12C.GL_CLAMP_TO_EDGE:
                return "CLAMP_TO_EDGE";
            case GL13C.GL_CLAMP_TO_BORDER:
                return "CLAMP_TO_BORDER";
            case GL14C.GL_MIRRORED_REPEAT:
                return "MIRRORED_REPEAT";
            case GL15C.GL_DYNAMIC_DRAW:
                return "DYNAMIC_DRAW";
            case GL15C.GL_STATIC_DRAW:
                return "STATIC_DRAW";
            case GL20C.GL_COMPILE_STATUS:
                return "COMPILE_STATUS";
            case GL20C.GL_LINK_STATUS:
                return "LINK_STATUS";

            default:
                return "unknown" + code;
        }
    }

    /**
     * Interpolate linearly between (or extrapolate linearly from) 2
     * single-precision values.
     * <p>
     * No rounding error is introduced when y0==y1.
     *
     * @param t the weight given to {@code y1}
     * @param y0 the function value at t=0
     * @param y1 the function value at t=1
     * @return the interpolated function value
     */
    public static float lerp(float t, float y0, float y1) {
        float result;
        if (y0 == y1) {
            result = y0;
        } else {
            float u = 1f - t;
            result = u * y0 + t * y1;
        }

        return result;
    }

    /**
     * Interpolate linearly between (or extrapolate linearly from) 2 vectors.
     * <p>
     * No rounding error is introduced when v1==v2.
     *
     * @param t the weight given to {@code v1}
     * @param v0 the function value at t=0 (not null, unaffected unless it's
     * also {@code storeResult})
     * @param v1 the function value at t=1 (not null, unaffected unless it's
     * also {@code storeResult})
     * @param storeResult storage for the result (modified if not null, may be
     * {@code v0} or {@code v1})
     * @return the interpolated value (either {@code storeResult} or a new
     * instance)
     */
    public static Vector3f lerp(
            float t, Vector3fc v0, Vector3fc v1, Vector3f storeResult) {
        Vector3f result = (storeResult == null) ? new Vector3f() : storeResult;

        float x = lerp(t, v0.x(), v1.x());
        float y = lerp(t, v0.y(), v1.y());
        float z = lerp(t, v0.z(), v1.z());
        result.set(x, y, z);

        return result;
    }

    /**
     * Load raw bytes from the named classpath resource.
     *
     * @param resourceName the name of the resource (not null)
     * @return a new array
     */
    public static ByteBuffer loadResourceAsBytes(String resourceName) {
        // Read the resource to determine its size in bytes:
        InputStream inputStream = Utils.class.getResourceAsStream(resourceName);
        if (inputStream == null) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("resource not found:  " + q);
        }
        int totalBytes = 0;
        byte[] tmpArray = new byte[4096];
        try {
            while (true) {
                int numBytesRead = inputStream.read(tmpArray);
                if (numBytesRead < 0) {
                    break;
                }
                totalBytes += numBytesRead;
            }
            inputStream.close();

        } catch (IOException exception) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("failed to read resource " + q);
        }
        ByteBuffer result = Jolt.newDirectByteBuffer(totalBytes);

        // Read the resource again to fill the buffer with data:
        inputStream = Utils.class.getResourceAsStream(resourceName);
        if (inputStream == null) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("resource not found:  " + q);
        }
        try {
            while (true) {
                int numBytesRead = inputStream.read(tmpArray);
                if (numBytesRead < 0) {
                    break;

                } else if (numBytesRead == tmpArray.length) {
                    result.put(tmpArray);

                } else {
                    for (int i = 0; i < numBytesRead; ++i) {
                        byte b = tmpArray[i];
                        result.put(b);
                    }
                }
            }
            inputStream.close();

        } catch (IOException exception) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("failed to read resource " + q);
        }

        result.flip();
        return result;
    }

    /**
     * Load an AWT BufferedImage from the named classpath resource.
     *
     * @param resourceName the name of the resource (not null)
     * @return a new object
     */
    public static BufferedImage loadResourceAsImage(String resourceName) {
        InputStream inputStream = Utils.class.getResourceAsStream(resourceName);
        if (inputStream == null) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("resource not found:  " + q);
        }

        ImageIO.setUseCache(false);

        try {
            BufferedImage result = ImageIO.read(inputStream);
            return result;

        } catch (IOException exception) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("unable to read " + q);
        }
    }

    /**
     * Load UTF-8 text from the named resource.
     *
     * @param resourceName the name of the classpath resource to load (not null)
     * @return the text (possibly multiple lines)
     */
    public static String loadResourceAsString(String resourceName) {
        InputStream inputStream = Utils.class.getResourceAsStream(resourceName);
        if (inputStream == null) {
            String q = MyString.quote(resourceName);
            throw new RuntimeException("resource not found:  " + q);
        }

        Scanner scanner
                = new Scanner(inputStream, StandardCharsets.UTF_8);
        String result = scanner.useDelimiter("\\A").next();

        return result;
    }

    /**
     * Find the maximum of some int values.
     *
     * @param iValues the input values
     * @return the most positive value
     * @see java.util.Collections#max(java.util.Collection)
     * @see java.lang.Math#max(int, int)
     */
    public static int maxInt(int... iValues) {
        int result = Integer.MIN_VALUE;
        for (int iValue : iValues) {
            if (iValue > result) {
                result = iValue;
            }
        }

        return result;
    }

    /**
     * Return the least non-negative value congruent with the input value with
     * respect to the specified modulus.
     * <p>
     * This differs from remainder for negative input values. For instance,
     * modulo(-1f, 4f) == 3f, while -1f % 4f == -1f.
     *
     * @param fValue the input value
     * @param modulus (&gt;0)
     * @return fValue MOD modulus (&lt;modulus, &ge;0)
     */
    public static float modulo(float fValue, float modulus) {
        assert modulus > 0f : modulus;

        float remainder = fValue % modulus;
        float result;
        if (fValue >= 0) {
            result = remainder;
        } else {
            result = (remainder + modulus) % modulus;
        }

        assert result >= 0f : result;
        assert result < modulus : result;
        return result;
    }

    /**
     * Return the least non-negative value congruent with the input value with
     * respect to the specified modulus.
     * <p>
     * This differs from remainder for negative input values. For instance,
     * modulo(-1, 4) == 3, while -1 % 4 == -1.
     *
     * @param iValue the input value
     * @param modulus (&gt;0)
     * @return iValue MOD modulus (&lt;modulus, &ge;0)
     */
    public static int modulo(int iValue, int modulus) {
        assert Validate.positive(modulus, "modulus");

        int remainder = iValue % modulus;
        int result;
        if (iValue >= 0) {
            result = remainder;
        } else {
            result = (remainder + modulus) % modulus;
        }

        assert result >= 0f : result;
        assert result < modulus : result;
        return result;
    }

    /**
     * Enable or disable the specified OpenGL capability.
     *
     * @param capability the numeric code for the capability
     * @param newState the desired state (true to enable, false to disable)
     */
    public static void setOglCapability(int capability, boolean newState) {
        if (newState) {
            GL11C.glEnable(capability);
            checkForOglError();
        } else {
            GL11C.glDisable(capability);
            checkForOglError();
        }
    }

    /**
     * Standardize a rotation angle to the range [-Pi, Pi).
     *
     * @param angle the input angle (in radians)
     * @return the standardized angle (in radians, &lt;Pi, &ge;-Pi)
     */
    public static float standardizeAngle(float angle) {
        assert Float.isFinite(angle);

        float result = modulo(angle, Constants.twoPi);
        if (result >= Jolt.JPH_PI) {
            result -= Constants.twoPi;
        }

        assert result >= -Jolt.JPH_PI : result;
        assert result < Jolt.JPH_PI : result;
        return result;
    }

    /**
     * Copy the specified FloatBuffer to an array.
     *
     * @param buffer the buffer to copy (not null, unaffected)
     * @return a new array (not null)
     */
    public static float[] toArray(FloatBuffer buffer) {
        float[] array = new float[buffer.limit()];
        for (int i = 0; i < buffer.limit(); ++i) {
            array[i] = buffer.get(i);
        }

        return array;
    }

    /**
     * Convert a BufferedImage to a {@code FloatBuffer} of heights.
     *
     * @param image the image to use (not null, unaffected)
     * @param maxHeight the vertical scaling factor
     * @return a new direct buffer containing values in the range [0,
     * maxHeight], one value for each pixel in the image
     */
    public static FloatBuffer toHeightBuffer(
            BufferedImage image, float maxHeight) {
        int imageWidth = image.getWidth();
        int imageHeight = image.getHeight();
        int numSamples = imageWidth * imageHeight;
        FloatBuffer result = Jolt.newDirectFloatBuffer(numSamples);

        int floatIndex = 0;
        for (int y = 0; y < imageHeight; ++y) {
            for (int x = 0; x < imageWidth; ++x) {
                int srgb = image.getRGB(x, y);
                double red = ((srgb >> 16) & 0xFF) / 255.0;
                double green = ((srgb >> 8) & 0xFF) / 255.0;
                double blue = (srgb & 0xFF) / 255.0;

                // linearize the pixel's color
                red = Math.pow(red, 2.2);
                green = Math.pow(green, 2.2);
                blue = Math.pow(blue, 2.2);

                double height = 0.299 * red + 0.587 * green + 0.114 * blue;
                result.put(floatIndex, maxHeight * (float) height);

                ++floatIndex;
            }
        }

        return result;
    }

    /**
     * Copy the specified JOML quaternion to a new Jolt-JNI quaternion.
     *
     * @param joml the JOML quaternion to copy (not {@code null}, unaffected)
     * @return a new Jolt-JNI quaternion
     */
    public static Quat toJoltQuaternion(Quaternionfc joml) {
        Quat result = new Quat(joml.x(), joml.y(), joml.z(), joml.w());
        return result;
    }

    /**
     * Copy the specified JOML quaternion to a Jolt-JNI quaternion.
     *
     * @param joml the JOML quaternion to copy (not {@code null}, unaffected)
     * @param storeResult storage for the result (modified if not {@code null})
     * @return the Jolt-JNI quaternion (either {@code storeResult} or a new
     * quaternion, not {@code null})
     */
    public static Quat toJoltQuaternion(Quaternionfc joml, Quat storeResult) {
        Quat result = (storeResult == null) ? new Quat() : storeResult;
        result.set(joml.x(), joml.y(), joml.z(), joml.w());
        return result;
    }

    /**
     * Copy the specified JOML vector to a new Jolt-JNI vector.
     *
     * @param joml the JOML vector to copy (not null, unaffected)
     * @return a new Jolt-JNI vector (not null)
     */
    public static Vec3 toJoltVector(Vector3fc joml) {
        Vec3 result = new Vec3(joml.x(), joml.y(), joml.z());
        return result;
    }

    /**
     * Copy the specified JOML vector to a Jolt vector.
     *
     * @param joml the JOML vector to copy (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a new Jolt vector (either {@code storeResult} or a new vector,
     * not null)
     */
    public static Vec3 toJoltVector(Vector3fc joml, Vec3 storeResult) {
        Vec3 result = (storeResult == null) ? new Vec3() : storeResult;
        result.set(joml.x(), joml.y(), joml.z());
        return result;
    }

    /**
     * Copy the specified Jolt quaternion to a new JOML quaternion.
     *
     * @param jolt the Jolt quaternion to copy (not null, unaffected)
     * @return a new JOML quaternion
     */
    public static Quaternionf toJomlQuaternion(Quat jolt) {
        Quaternionf result = new Quaternionf(
                jolt.getX(), jolt.getY(), jolt.getZ(), jolt.getW());
        return result;
    }

    /**
     * Copy the specified Jolt location vector to a new JOML vector.
     *
     * @param jolt the Jolt vector to copy (not null, unaffected)
     * @return a new JOML vector (not null)
     */
    public static Vector3f toJomlVector(RVec3Arg jolt) {
        return new Vector3f(jolt.x(), jolt.y(), jolt.z());
    }

    /**
     * Copy the specified Jolt vector to a new JOML vector.
     *
     * @param jolt the Jolt vector to copy (not null, unaffected)
     * @return a new JOML vector (not null)
     */
    public static Vector3f toJomlVector(Vec3Arg jolt) {
        return new Vector3f(jolt.getX(), jolt.getY(), jolt.getZ());
    }

    /**
     * Copy the specified Jolt vector to a JOML vector.
     *
     * @param jolt the Jolt vector to copy (not null, unaffected)
     * @param storeResult storage for the result (modified if not null)
     * @return a JOML vector (either {@code storeResult} or a new vector, not
     * null)
     */
    public static Vector3f toJomlVector(Vec3Arg jolt, Vector3f storeResult) {
        return new Vector3f(jolt.getX(), jolt.getY(), jolt.getZ());
    }

    /**
     * Convert an sRGB color string to a color in the linear colorspace.
     *
     * @param hexString the input color (hexadecimal string with red channel in
     * the most-significant byte, alpha channel in the least significant byte)
     * @return a new vector (red channel in the X component, alpha channel in
     * the W component)
     *
     * @throws NumberFormatException if {@code hexString} fails to parse
     */
    public static Vector4f toLinearColor(String hexString) {
        int srgbColor = Integer.parseUnsignedInt(hexString, 16);

        double red = ((srgbColor >> 24) & 0xFF) / 255.0;
        double green = ((srgbColor >> 16) & 0xFF) / 255.0;
        double blue = ((srgbColor >> 8) & 0xFF) / 255.0;

        // linearize the color channels
        float r = (float) Math.pow(red, 2.2);
        float g = (float) Math.pow(green, 2.2);
        float b = (float) Math.pow(blue, 2.2);

        float a = (srgbColor & 0xFF) / 255f;

        return new Vector4f(r, g, b, a);
    }

    /**
     * Copy the specified JOML vector to a new Jolt-JNI location vector.
     *
     * @param joml the JOML vector to copy (not {@code null}, unaffected)
     * @return a new Jolt-JNI vector
     */
    public static RVec3 toLocationVector(Vector3fc joml) {
        RVec3 result = new RVec3(joml.x(), joml.y(), joml.z());
        return result;
    }

    /**
     * Convert the specified vector from Cartesian coordinates to spherical
     * coordinates (r, theta, phi) per ISO 80000.
     * <p>
     * In particular:
     * <ul>
     * <li>{@code r} is a distance measured from the origin. It ranges from 0 to
     * infinity and is stored in the first (X) vector component.
     *
     * <li>{@code theta} is the polar angle, measured (in radians) from the +Z
     * axis. It ranges from 0 to PI and is stored in the 2nd (Y) vector
     * component.
     *
     * <li>{@code phi} is the azimuthal angle, measured (in radians) from the +X
     * axis to the projection of the vector onto the X-Y plane. It ranges from
     * -PI to PI and is stored in the 3rd (Z) vector component.
     * </ul>
     *
     * @param vec the vector to convert (not null, modified)
     */
    public static void toSpherical(Vec3 vec) {
        double xx = vec.getX();
        double yy = vec.getY();
        double zz = vec.getZ();
        double sumOfSquares = xx * xx + yy * yy;
        double rxy = Math.sqrt(sumOfSquares);
        double theta = Math.atan2(yy, xx);
        sumOfSquares += zz * zz;
        double phi = Math.atan2(rxy, zz);
        double rr = Math.sqrt(sumOfSquares);

        vec.setX((float) rr);    // distance from origin
        vec.setY((float) theta); // polar angle
        vec.setZ((float) phi);   // azimuthal angle
    }
}
