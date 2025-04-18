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

/**
 * A uniform variable whose value is the same for every Geometry and whose name
 * is the same in every ShaderProgram that uses it.
 *
 * @author Stephen Gold sgold@sonic.net
 */
abstract class GlobalUniform {
    // *************************************************************************
    // fields

    /**
     * name of the uniform variable
     */
    final private String variableName;
    // *************************************************************************
    // constructors

    /**
     * Instantiate a new global uniform.
     *
     * @param name the desired name (not null, not blank)
     */
    protected GlobalUniform(String name) {
        Validate.nonBlank(name, "name");
        this.variableName = name;
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Return the name of the variable.
     *
     * @return the name string (not null, not empty)
     */
    String getVariableName() {
        return variableName;
    }

    /**
     * Send the current value to the specified program.
     *
     * @param program the program to update (not null)
     */
    abstract void sendValueTo(ShaderProgram program);

    /**
     * Update the uniform's value.
     */
    abstract void updateValue();
}
