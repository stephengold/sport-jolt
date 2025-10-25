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
package com.github.stephengold.sportjolt.physics;

import com.github.stephengold.joltjni.PhysicsSystem;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import org.apache.logging.log4j.util.TriConsumer;

/**
 * An app to visualize 3-D physics using functional interfaces.
 */
public class FunctionalPhysicsApp
        extends BasePhysicsApp
        implements PhysicsTickListener {
    // *************************************************************************
    // fields

    /**
     * callback to calculate how much to advance the simulation before rendering
     * the next frame
     */
    private BiFunction<BasePhysicsApp, Float, Float> advanceAmount;
    /**
     * callback to initialize the graphics engine before the main update loop
     * begins
     */
    private Consumer<BasePhysicsApp> initialize;
    /**
     * callback to populate the PhysicsSystem during initialization
     */
    private Consumer<BasePhysicsApp> populateSystem;
    /**
     * callback invoked after each frame is rendered
     */
    private Consumer<BasePhysicsApp> postRender;
    /**
     * callback invoked before each frame is rendered
     */
    private Consumer<BasePhysicsApp> preRender;
    /**
     * callback to create the PhysicsSystem during initialization
     */
    private Function<BasePhysicsApp, PhysicsSystem> createSystem;
    /**
     * callback invoked after each simulation step
     */
    private TriConsumer<BasePhysicsApp, PhysicsSystem, Float> postPhysicsTick;
    /**
     * callback invoked before each simulation step
     */
    private TriConsumer<BasePhysicsApp, PhysicsSystem, Float> prePhysicsTick;
    // *************************************************************************
    // constructors

    /**
     * Explicit no-arg constructor to avoid javadoc warnings from JDK 18+.
     */
    public FunctionalPhysicsApp() {
        // do nothing
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Replace the callback to calculate how much to advance the simulation
     * before rendering the next frame.
     *
     * @param function the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setAdvanceAmount(
            BiFunction<BasePhysicsApp, Float, Float> function) {
        this.advanceAmount = function;
        return this;
    }

    /**
     * Replace the function to create the PhysicsSystem during initialization.
     *
     * @param function the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setCreateSystem(
            Function<BasePhysicsApp, PhysicsSystem> function) {
        this.createSystem = function;
        return this;
    }

    /**
     * Replace the function invoked to initialize the graphics engine before the
     * main update loop begins.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setInitialize(
            Consumer<BasePhysicsApp> consumer) {
        this.initialize = consumer;
        return this;
    }

    /**
     * Replace the function to populate the PhysicsSystem during initialization.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setPopulateSystem(
            Consumer<BasePhysicsApp> consumer) {
        this.populateSystem = consumer;
        return this;
    }

    /**
     * Replace the function to invoked after the simulation is stepped.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setPostPhysicsTick(
            TriConsumer<BasePhysicsApp, PhysicsSystem, Float> consumer) {
        this.postPhysicsTick = consumer;
        return this;
    }

    /**
     * Replace the function invoked after each frame is rendered.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setPostRender(
            Consumer<BasePhysicsApp> consumer) {
        this.postRender = consumer;
        return this;
    }

    /**
     * Replace the function to invoked before each simulation step.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setPrePhysicsTick(
            TriConsumer<BasePhysicsApp, PhysicsSystem, Float> consumer) {
        this.prePhysicsTick = consumer;
        return this;
    }

    /**
     * Replace the function invoked before each frame is rendered.
     *
     * @param consumer the function to use, or {@code null} for none
     * @return the modified application, for chaining
     */
    public FunctionalPhysicsApp setPreRender(
            Consumer<BasePhysicsApp> consumer) {
        this.preRender = consumer;
        return this;
    }
    // *************************************************************************
    // BasePhysicsApp methods

    /**
     * Create the PhysicsSystem during initialization.
     *
     * @return a new object
     */
    @Override
    final public PhysicsSystem createSystem() {
        PhysicsSystem result;
        if (createSystem == null) {
            result = super.createSystem();
        } else {
            result = createSystem.apply(this);
        }

        // Always add this object as a tick listener:
        addTickListener(this);

        return result;
    }

    /**
     * Callback invoked before the main update loop begins.
     */
    @Override
    final public void initialize() {
        super.initialize();
        if (initialize != null) {
            initialize.accept(this);
        }
    }

    /**
     * Callback to populate the PhysicsSystem.
     */
    @Override
    final public void populateSystem() {
        if (populateSystem != null) {
            populateSystem.accept(this);
        }
    }

    /**
     * Callback invoked during each iteration of the render loop.
     */
    @Override
    final public void render() {
        if (preRender != null) {
            preRender.accept(this);
        }
        super.render();
        if (postRender != null) {
            postRender.accept(this);
        }
    }

    /**
     * Advance the physics simulation by the specified amount. Invoked during
     * each update.
     *
     * @param wallClockSeconds the elapsed wall-clock time since the previous
     * invocation of {@code updatePhysics} (in seconds, &ge;0)
     */
    @Override
    final public void updatePhysics(float wallClockSeconds) {
        if (advanceAmount == null) {
            super.updatePhysics(wallClockSeconds);
        } else {
            float simulateSeconds = advanceAmount.apply(this, wallClockSeconds);
            super.updatePhysics(simulateSeconds);
        }
    }
    // *************************************************************************
    // PhysicsTickListener methods

    /**
     * Callback invoked (by Sport-Jolt, not by Jolt Physics) after the system
     * has been stepped.
     *
     * @param system the system that was just stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    final public void physicsTick(PhysicsSystem system, float timeStep) {
        if (postPhysicsTick != null) {
            postPhysicsTick.accept(this, system, timeStep);
        }
    }

    /**
     * Callback invoked (by Sport-Jolt, not by Jolt Physics) before the system
     * is stepped.
     *
     * @param system the system that's about to be stepped (not null)
     * @param timeStep the duration of the simulation step (in seconds, &ge;0)
     */
    @Override
    final public void prePhysicsTick(PhysicsSystem system, float timeStep) {
        if (prePhysicsTick != null) {
            prePhysicsTick.accept(this, system, timeStep);
        }
    }
}
