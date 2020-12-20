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

    /// <summary>
    /// `Web` is the top-level static class for the 3D WebView plugin.
    /// It contains static methods for configuring the module and creating resources.
    /// </summary>
    public static class Web {

        /// <summary>
        /// Clears all data that persists between webview instances,
        /// including cookies, storage, and cached resources.
        /// </summary>
        /// <remarks>
        /// On Windows and macOS, this method can only be called prior to
        /// creating any webviews.
        /// </remarks>
        public static void ClearAllData() {

            _pluginFactory.GetPlugin().ClearAllData();
        }

        /// <summary>
        /// Creates a material and texture that the webview can use for rendering.
        /// </summary>
        /// <remarks>
        /// Note that the `WebViewPrefab` takes care of material creation for you, so you only need
        /// to call this method directly if you create `IWebView` instances outside of a prefab using
        /// `Web.CreateWebView()`.
        ///
        /// Material creation occurs asynchronously
        /// in order to allow textures to be created on the render thread, and the
        /// provided callback is called once the material has been created.
        /// </remarks>
        public static void CreateMaterial(Action<Material> callback) {

            _pluginFactory.GetPlugin().CreateMaterial(callback);
        }

        /// <summary>
        /// Like `CreateMaterial`, except it creates a material that a webview
        /// can use for rendering video. If the platform doesn't need a separate
        /// material and texture for video, this method returns `null`.
        /// </summary>
        /// <remarks>
        /// Android and iOS use a separate material and texture for video rendering,
        /// whereas standalone Windows and macOS do not.
        /// </remarks>
        public static void CreateVideoMaterial(Action<Material> callback) {

            _pluginFactory.GetPlugin().CreateVideoMaterial(callback);
        }

        /// <summary>
        /// Creates a special texture that the webview can use for rendering.
        /// </summary>
        /// <remarks>
        /// Note that the `WebViewPrefab` takes care of texture creation for you, so you only need
        /// to call this method directly if you create `IWebView` instances outside of a prefab using
        /// `Web.CreateWebView()`.
        ///
        /// Texture creation occurs asynchronously
        /// in order to allow textures to be created on the render thread, and the
        /// provided callback is called once the texture has been created.
        /// </remarks>
        public static void CreateTexture(float width, float height, Action<Texture2D> callback) {

            _pluginFactory.GetPlugin().CreateTexture(width, height, callback);
        }

        /// <summary>
        /// Create a new webview in a platform-agnostic way.
        /// </summary>
        /// <remarks>
        /// Note that `WebViewPrefab` takes care of creating an `IWebView`
        /// instance for you, so you only need to call this method directly
        /// if you need to create `IWebView` instances outside of a prefab.
        /// </remarks>
        public static IWebView CreateWebView() {

            return _pluginFactory.GetPlugin().CreateWebView();
        }

        /// <summary>
        /// Controls whether data like cookies, localStorage, and cached resources
        /// is persisted between webview instances. The default is `true`, but this
        /// can be set to `false` to achieve an "incognito mode".
        /// </summary>
        /// <remarks>
        /// On Windows and macOS, this method can only be called prior to
        /// creating any webviews.
        /// </remarks>
        public static void SetStorageEnabled(bool enabled) {

            _pluginFactory.GetPlugin().SetStorageEnabled(enabled);
        }

        /// <summary>
        /// By default, webviews use a User-Agent that looks that of a desktop
        /// computer so that servers return the desktop versions of websites.
        /// If you instead want the mobile versions of websites, you can invoke
        /// this method with `true` to use the User-Agent for a mobile device.
        /// </summary>
        /// <remarks>
        /// On Windows and macOS, this method can only be called prior to
        /// creating any webviews.
        /// </remarks>
        public static void SetUserAgent(bool mobile) {

            _pluginFactory.GetPlugin().SetUserAgent(mobile);
        }

        /// <summary>
        /// Configures the module to use a custom User-Agent string.
        /// </summary>
        /// <remarks>
        /// On Windows and macOS, this method can only be called prior to
        /// creating any webviews.
        /// </remarks>
        public static void SetUserAgent(string userAgent) {

            _pluginFactory.GetPlugin().SetUserAgent(userAgent);
        }

        static internal void SetPluginFactory(WebPluginFactory pluginFactory) {

            _pluginFactory = pluginFactory;
        }

        static WebPluginFactory _pluginFactory = new WebPluginFactory();
    }
}
