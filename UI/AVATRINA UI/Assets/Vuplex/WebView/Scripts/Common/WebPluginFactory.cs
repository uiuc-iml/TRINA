/**
* Copyright 2019 Vuplex Inc. All rights reserved.
*
* Licensed under the Vuplex Commercial Software Library License, you may
* not use this file except in compliance with the License. You may obtain
* a copy of the License at
*
*     https://vuplex.com/commercial-library-license
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
using System;
using UnityEngine;

namespace Vuplex.WebView {

    class WebPluginFactory {

        public virtual IWebPlugin GetPlugin() {

            #if UNITY_STANDALONE_WIN
                if (_windowsPlugin != null) {
                    return _windowsPlugin;
                }
                throw new Exception("The 3D WebView for Windows plugin is not currently installed." + MORE_INFO_TEXT);
            #elif UNITY_STANDALONE_OSX
                if (_macPlugin != null) {
                    return _macPlugin;
                }
                throw new Exception("The 3D WebView for macOS plugin is not currently installed." + MORE_INFO_TEXT);
            #elif (UNITY_IOS || UNITY_ANDROID) && UNITY_EDITOR
                #if UNITY_EDITOR_WIN
                    if (_windowsPlugin != null) {
                        return _windowsPlugin;
                    }
                    Debug.LogWarning("The 3D WebView for Windows plugin is not currently installed, so the MockWebView will be used while running in the editor." + MORE_INFO_TEXT);
                    return _mockPlugin;
                #elif UNITY_EDITOR_OSX
                    if (_macPlugin != null) {
                        return _macPlugin;
                    }
                    Debug.LogWarning("The 3D WebView for macOS plugin is not currently installed, so the MockWebView will be used while running in the editor." + MORE_INFO_TEXT);
                    return _mockPlugin;
                #else
                    Debug.LogWarning("There is not currently a 3D WebView plugin for the current editor platform, so the MockWebView will be used while running in the editor." + MORE_INFO_TEXT);
                    return _mockPlugin;
                #endif
            #elif UNITY_IOS
                if (_iosPlugin != null) {
                    return _iosPlugin;
                }
                throw new Exception("The 3D WebView for iOS plugin is not currently installed." + MORE_INFO_TEXT);
            #elif UNITY_ANDROID
                if (_androidPlugin != null) {
                    return _androidPlugin;
                }
                throw new Exception("The 3D WebView for Android plugin is not currently installed." + MORE_INFO_TEXT);
            #else
                throw new Exception("This version of 3D WebView does not support the current build platform." + MORE_INFO_TEXT);
            #endif
        }

        public static void RegisterAndroidPlugin(IWebPlugin plugin) {

            _androidPlugin = plugin;
        }

        public static void RegisterIOSPlugin(IWebPlugin plugin) {

            _iosPlugin = plugin;
        }

        public static void RegisterMacPlugin(IWebPlugin plugin) {

            _macPlugin = plugin;
        }

        public static void RegisterMockPlugin(IWebPlugin plugin) {

            _mockPlugin = plugin;
        }

        public static void RegisterWindowsPlugin(IWebPlugin plugin) {

            _windowsPlugin = plugin;
        }

        static IWebPlugin _androidPlugin;
        static IWebPlugin _iosPlugin;
        static IWebPlugin _macPlugin;
        static IWebPlugin _mockPlugin = MockWebPlugin.Instance;
        const string MORE_INFO_TEXT = " For more info, please visit https://developer.vuplex.com";
        static IWebPlugin _windowsPlugin;
    }
}
