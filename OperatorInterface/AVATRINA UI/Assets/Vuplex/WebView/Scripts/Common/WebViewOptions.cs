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
namespace Vuplex.WebView {

    /// <summary>
    /// Options that can be passed to the `WebViewPrefab.Instantiate()` to alter the behavior of
    /// the WebView created.
    /// </summary>
    public struct WebViewOptions {

        /// <summary>
        /// Videos are enabled by default, but you can disable them with this option
        /// if you know you will not need to render videos. Setting this option to `true` will prevent the prefab
        /// from allocating a second texture for video and prevent it from allocating
        /// resources for video playback.
        /// </summary>
        public bool disableVideo;
    }
}
