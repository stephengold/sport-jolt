/*
 Copyright (c) 2014-2025 Stephen Gold and Yanis Boudiaf

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
import com.github.stephengold.joltjni.readonly.QuatArg;
import com.github.stephengold.joltjni.readonly.RVec3Arg;
import com.github.stephengold.joltjni.readonly.Vec3Arg;
import java.util.Collection;
import org.joml.Vector3fc;

/**
 * Utility methods to throw exceptions for invalid method arguments.
 * <p>
 * These methods are intended for checking the arguments of public/protected
 * methods in library classes. To check return values, or the arguments of
 * private/package methods, use assertions.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class Validate {
    // *************************************************************************
    // fields

    /**
     * If {@code true}, throw {@code NullPointerException} for null arguments.
     * Otherwise, throw {@code IllegalArgumentException}.
     */
    public static boolean throwNpe = true;
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private Validate() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Validate an axis index as a method argument.
     *
     * @param iValue the index to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [0, 2]
     */
    public static boolean axisIndex(int iValue, String description) {
        inRange(iValue, description, 0, Mesh.numAxes - 1);
        return true;
    }

    /**
     * Validate a finite single-precision value as a method argument.
     *
     * @param fValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is NaN or infinite
     */
    public static boolean finite(float fValue, String description) {
        if (!Float.isFinite(fValue)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + fValue);
            System.err.flush();
            String message = what + " must be a finite number.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a finite {@code RVec3Arg} as a method argument.
     *
     * @param vector the vector to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the vector has a NaN or infinite
     * component
     * @throws NullPointerException or IllegalArgumentException if the vector is
     * {@code null}
     */
    public static boolean finite(RVec3Arg vector, String description) {
        nonNull(vector, description);

        if (!vector.isFinite()) {
            String what;
            if (description == null) {
                what = "Vec3Arg argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + vector);
            System.err.flush();
            String message = what + " must have all components finite.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a finite {@code Vec3Arg} as a method argument.
     *
     * @param vector the vector to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the vector has a NaN or infinite
     * component
     * @throws NullPointerException or IllegalArgumentException if the vector is
     * {@code null}
     */
    public static boolean finite(Vec3Arg vector, String description) {
        nonNull(vector, description);

        if (!vector.isFinite()) {
            String what;
            if (description == null) {
                what = "Vec3Arg argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + vector);
            System.err.flush();
            String message = what + " must have all components finite.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a finite {@code Vector3fc} as a method argument.
     *
     * @param vector the vector to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the vector has a NaN or infinite
     * component
     * @throws NullPointerException or IllegalArgumentException if the vector is
     * {@code null}
     */
    public static boolean finite(Vector3fc vector, String description) {
        nonNull(vector, description);

        if (!vector.isFinite()) {
            String what;
            if (description == null) {
                what = "Vector3fc argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + vector);
            System.err.flush();
            String message = what + " must have all components finite.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a finite double-precision value as a method argument.
     *
     * @param dValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is NaN or infinite
     */
    public static boolean finiteDouble(double dValue, String description) {
        if (!Double.isFinite(dValue)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + dValue);
            System.err.flush();
            String message = what + " must be a finite number.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-negative proper fraction as a method argument.
     *
     * @param fValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [0, 1]
     */
    public static boolean fraction(float fValue, String description) {
        inRange(fValue, description, 0f, 1f);
        return true;
    }

    /**
     * Validate a non-negative proper fraction as a method argument.
     *
     * @param dValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [0, 1]
     */
    public static boolean fractionDouble(double dValue, String description) {
        inRangeDouble(dValue, description, 0., 1.);
        return true;
    }

    /**
     * Validate a limited single-precision value as a method argument.
     *
     * @param fValue the value to validate
     * @param description a description of the argument
     * @param min the smallest valid value (&le;max)
     * @param max the largest valid value (&ge;max)
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [min,
     * max]
     */
    public static boolean inRange(
            float fValue, String description, float min, float max) {
        if (!(fValue >= min)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + fValue);
            System.err.flush();
            String message = String.format(
                    "%s must be greater than or equal to %f.", what, min);
            throw new IllegalArgumentException(message);
        }

        if (!(fValue <= max)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + fValue);
            System.err.flush();
            String message = String.format(
                    "%s must be less than or equal to %f.", what, max);
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a limited integer value as a method argument.
     *
     * @param iValue the value to validate
     * @param description a description of the argument
     * @param min the smallest valid value (&le;max)
     * @param max the largest valid value (&ge;max)
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [min,
     * max]
     * <p>
     * Compare with {@code Objects.checkIndex()} in Java 9.
     */
    public static boolean inRange(
            int iValue, String description, int min, int max) {
        if (iValue < min) {
            String what;
            if (description == null) {
                what = "int argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + iValue);
            System.err.flush();
            String message = String.format(
                    "%s must be greater than or equal to %d.", what, min);
            throw new IllegalArgumentException(message);
        }

        if (iValue > max) {
            String what;
            if (description == null) {
                what = "int argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + iValue);
            System.err.flush();
            String message = String.format(
                    "%s must be less than or equal to %d.", what, max);
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a limited double-precision value as a method argument.
     *
     * @param dValue the value to validate
     * @param description a description of the argument
     * @param min the smallest valid value (&le;max)
     * @param max the largest valid value (&ge;max)
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the specified
     * range
     */
    public static boolean inRangeDouble(
            double dValue, String description, double min, double max) {
        if (!(dValue >= min)) {
            String what;
            if (description == null) {
                what = "double argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + dValue);
            System.err.flush();
            String message = String.format(
                    "%s must be greater than or equal to %f.", what, min);
            throw new IllegalArgumentException(message);
        }

        if (!(dValue <= max)) {
            String what;
            if (description == null) {
                what = "double argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + dValue);
            System.err.flush();
            String message = String.format(
                    "%s must be less than or equal to %f.", what, max);
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null, non-blank string as a method argument.
     *
     * @param string the string to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the String is
     * {@code null}
     * @throws IllegalArgumentException if the String is blank
     */
    public static boolean nonBlank(String string, String description) {
        nonNull(string, description);

        if (string.isBlank()) {
            String what;
            if (description == null) {
                what = "String argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + MyString.quote(string));
            System.err.flush();
            String message = what + " must not be blank.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null, non-empty collection as a method argument.
     *
     * @param collection the collection to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the
     * collection is null
     * @throws IllegalArgumentException if the collection is empty
     */
    public static boolean nonEmpty(
            Collection<?> collection, String description) {
        nonNull(collection, description);

        if (collection.isEmpty()) {
            String what;
            if (description == null) {
                what = "Collection argument";
            } else {
                what = description;
            }
            String message = what + " must not be empty.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null, non-empty float array as a method argument.
     *
     * @param array the array to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the array is
     * null
     * @throws IllegalArgumentException if the array has zero length
     */
    public static boolean nonEmpty(float[] array, String description) {
        nonNull(array, description);

        if (array.length == 0) {
            String what;
            if (description == null) {
                what = "float[] argument";
            } else {
                what = description;
            }
            String message = what + " must not be empty.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null, non-empty object array as a method argument.
     *
     * @param array the array to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the array is
     * null
     * @throws IllegalArgumentException if the array has zero length
     */
    public static boolean nonEmpty(Object[] array, String description) {
        nonNull(array, description);

        if (array.length == 0) {
            String what;
            if (description == null) {
                what = "Object[] argument";
            } else {
                what = description;
            }
            String message = what + " must not be empty.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null, non-empty string as a method argument.
     *
     * @param string the string to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the String is
     * null
     * @throws IllegalArgumentException if the String has zero length
     */
    public static boolean nonEmpty(String string, String description) {
        nonNull(string, description);

        if (string.isEmpty()) {
            String what;
            if (description == null) {
                what = "String argument";
            } else {
                what = description;
            }
            String message = what + " must not be empty.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-negative single-precision value as a method argument.
     *
     * @param fValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative or NaN
     */
    public static boolean nonNegative(float fValue, String description) {
        if (!(fValue >= 0f)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + fValue);
            System.err.flush();
            String message = what + " must not be negative or NaN.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-negative integer value as a method argument.
     *
     * @param iValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative
     */
    public static boolean nonNegative(int iValue, String description) {
        if (iValue < 0) {
            String what;
            if (description == null) {
                what = "int argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + iValue);
            System.err.flush();
            String message = what + " must not be negative.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-negative double-precision value as a method argument.
     *
     * @param dValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative or NaN
     */
    public static boolean nonNegativeDouble(float dValue, String description) {
        if (!(dValue >= 0.)) {
            String what;
            if (description == null) {
                what = "double argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + dValue);
            System.err.flush();
            String message = what + " must not be negative or NaN.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-null reference. In many situations, validation can be
     * omitted because the object in question is about to be dereferenced.
     * <p>
     * While it might seem more logical to throw an IllegalArgumentException in
     * the case of a method argument, the javadoc for NullPointerException says,
     * "Applications should throw instances of this class to indicate other
     * illegal uses of the null object." To throw an IllegalArgumentException
     * instead, set {@link #throwNpe} to false.
     * <p>
     * Compare with {@code java.util.Objects.requireNonNull()}.
     *
     * @param object the reference to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws NullPointerException or IllegalArgumentException if the reference
     * is {@code null}
     */
    public static boolean nonNull(Object object, String description) {
        if (object == null) {
            String what;
            if (description == null) {
                what = "Object argument";
            } else {
                what = description;
            }
            String message = what + " must not be null.";
            if (throwNpe) {
                throw new NullPointerException(message);
            } else {
                throw new IllegalArgumentException(message);
            }
        }

        return true;
    }

    /**
     * Validate a non-zero long value as a method argument.
     *
     * @param lValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is zero
     */
    public static boolean nonZero(long lValue, String description) {
        if (lValue == 0L) {
            String what;
            if (description == null) {
                what = "long argument";
            } else {
                what = description;
            }
            String message = what + " must not be zero.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a non-zero {@code QuatArg} as a method argument.
     *
     * @param quat the value to validate (unaffected)
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the quaternion equals (0,0,0,0) or
     * has a NaN or infinite component
     * @throws NullPointerException or IllegalArgumentException if the
     * quaternion is null
     */
    public static boolean nonZero(QuatArg quat, String description) {
        nonNull(quat, description);

        if (quat.isZero()) {
            String what;
            if (description == null) {
                what = "Quat argument";
            } else {
                what = description;
            }
            String message = what + " must not be zero.";
            throw new IllegalArgumentException(message);
        }

        if (!quat.isFinite()) {
            String what;
            if (description == null) {
                what = "Quat argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + quat);
            System.err.flush();
            String message
                    = what + " must not have a NaN or infinite component.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a positive single-precision value as a method argument.
     *
     * @param fValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative or zero or NaN
     */
    public static boolean positive(float fValue, String description) {
        if (!(fValue > 0f)) {
            String what;
            if (description == null) {
                what = "float argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + fValue);
            System.err.flush();
            String message = what + " must not be negative or zero or NaN.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a positive integer value as a method argument.
     *
     * @param iValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative or zero
     */
    public static boolean positive(int iValue, String description) {
        if (iValue <= 0) {
            String what;
            if (description == null) {
                what = "int argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + iValue);
            System.err.flush();
            String message = what + " must not be negative or zero.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a positive double-precision value as a method argument.
     *
     * @param dValue the value to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is negative or zero or NaN
     */
    public static boolean positiveDouble(double dValue, String description) {
        if (!(dValue > 0.)) {
            String what;
            if (description == null) {
                what = "double argument";
            } else {
                what = description;
            }
            System.out.flush();
            System.err.println(what + " = " + dValue);
            System.err.flush();
            String message = what + " must not be negative or zero or NaN.";
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate an arbitrary boolean-valued expression involving method
     * arguments.
     *
     * @param value the value of the expression (required to be {@code true})
     * @param what a description of the requirement
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is {@code false}
     */
    public static boolean require(boolean value, String what) {
        if (!value) {
            String message;
            if (what == null) {
                message = "";
            } else {
                message = "Must have " + what + ".";
            }
            throw new IllegalArgumentException(message);
        }

        return true;
    }

    /**
     * Validate a standardized angle as a method argument.
     *
     * @param fValue the angle to validate
     * @param description a description of the argument
     * @return {@code true} (for use in {@code assert} statements)
     * @throws IllegalArgumentException if the value is outside the range [-PI,
     * PI]
     */
    public static boolean standardAngle(float fValue, String description) {
        inRange(fValue, description, -Jolt.JPH_PI, Jolt.JPH_PI);
        return true;
    }
}
