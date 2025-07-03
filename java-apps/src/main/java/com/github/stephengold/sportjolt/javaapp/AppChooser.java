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
package com.github.stephengold.sportjolt.javaapp;

import com.github.stephengold.sportjolt.BaseApplication;
import com.github.stephengold.sportjolt.javaapp.demo.NewtonsCradle;
import com.github.stephengold.sportjolt.javaapp.demo.Pachinko;
import com.github.stephengold.sportjolt.javaapp.demo.ThousandCubes;
import com.github.stephengold.sportjolt.javaapp.image.ShapeImages;
import com.github.stephengold.sportjolt.javaapp.test.AssimpTest;
import com.github.stephengold.sportjolt.javaapp.test.CheckerboardTest;
import com.github.stephengold.sportjolt.javaapp.test.ClipspaceTest;
import com.github.stephengold.sportjolt.javaapp.test.DynamicMeshTest;
import com.github.stephengold.sportjolt.javaapp.test.IcosphereTest;
import com.github.stephengold.sportjolt.javaapp.test.MouseTest;
import com.github.stephengold.sportjolt.javaapp.test.MouseTest2;
import com.github.stephengold.sportjolt.javaapp.test.OctasphereTest;
import com.github.stephengold.sportjolt.javaapp.test.RainbowTest;
import com.github.stephengold.sportjolt.javaapp.test.SpriteTest;
import com.github.stephengold.sportjolt.javaapp.test.TextureTest;
import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.event.WindowEvent;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.WindowConstants;

/**
 * Choose a Sport-Jolt application to run.
 */
final class AppChooser extends JFrame {
    // *************************************************************************
    // new methods exposed

    /**
     * Main entry point for the AppChooser application.
     *
     * @param arguments array of command-line arguments (not null)
     */
    public static void main(String[] arguments) {
        Logger.getLogger("").setLevel(Level.WARNING);

        List<BaseApplication> apps = List.of(
                new AssimpTest(),
                new CheckerboardTest(),
                new ClipspaceTest(),
                new DynamicMeshTest(),
                new IcosphereTest(),
                new MouseTest(),
                new MouseTest2(),
                new NewtonsCradle(),
                new OctasphereTest(),
                new Pachinko(),
                new RainbowTest(),
                new ShapeImages(),
                new SpriteTest(),
                new TextureTest(),
                new ThousandCubes()
        );
        new AppChooser(apps);
    }
    // *************************************************************************
    // private methods

    /**
     * Select and run one Jolt-Sport app from the specified list.
     *
     * @param apps the list of apps to choose from (not null, unaffected)
     */
    private AppChooser(List<? extends BaseApplication> apps) {
        setTitle("Sport-Jolt AppChooser");
        setSize(500, 100);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        Container contentPane = getContentPane();

        // Add a ComboBox to select one app.
        JComboBox<String> comboBox = new JComboBox<>();
        for (BaseApplication app : apps) {
            String appName = app.getClass().getSimpleName();
            comboBox.addItem(appName);
        }
        contentPane.add(BorderLayout.CENTER, comboBox);

        // Add a JButton to start the selected app.
        JButton startButton = new JButton("Start the selected app");
        startButton.addActionListener(actionEvent -> {
            setVisible(false);
            int selectedIndex = comboBox.getSelectedIndex();
            apps.get(selectedIndex).start();
            dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
        });
        contentPane.add(BorderLayout.EAST, startButton);

        setVisible(true);
    }
}
