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

    interface IWebPlugin {

        void ClearAllData();

        void CreateTexture(float width, float height, Action<Texture2D> callback);

        void CreateMaterial(Action<Material> callback);

        void CreateVideoMaterial(Action<Material> callback);

        IWebView CreateWebView();

        void SetStorageEnabled(bool enabled);

        void SetUserAgent(bool mobile);

        void SetUserAgent(string userAgent);
    }
}
