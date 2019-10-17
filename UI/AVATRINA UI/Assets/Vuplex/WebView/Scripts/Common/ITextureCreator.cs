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
    /// Interface for an object that creates a special texture that the webview can use for rendering.
    /// </summary>
    /// <remarks>
    /// Note that the `WebViewPrefab` takes care of using this interface for you, so you only need
    /// to deal with it directly if you create `IWebView` instances outside of a prefab using
    /// `WebViewFactory.GetInstance()`.
    /// </remarks>
    interface ITextureCreator {

        /// <summary>
        /// Creates a special texture that the webview can use for rendering, using
        /// the given dimensions in Unity units.
        /// </summary>
        /// <remarks>
        /// Texture creation occurs asynchronously
        /// in order to allow textures to be created on the render thread, and the
        /// provided callback is called once the texture has been created.
        /// </remarks>
        void CreateTexture(float width, float height, Action<Texture2D> callback);
    }
}

