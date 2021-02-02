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
#if UNITY_ANDROID && !UNITY_EDITOR
using System;
using UnityEngine;

#if UNITY_2017_2_OR_NEWER
    using UnityEngine.XR;
#endif // UNITY_2017_2_OR_NEWER

namespace Vuplex.WebView {

    class AndroidWebPlugin : MonoBehaviour, IWebPlugin {

        public static AndroidWebPlugin Instance {
            get {
                if (_instance == null) {
                    _instance = (AndroidWebPlugin) new GameObject("AndroidWebPlugin").AddComponent<AndroidWebPlugin>();
                }
                return _instance;
            }
        }

        public void ClearAllData() {

            AndroidWebView.ClearAllData();
        }

        public void CreateTexture(float width, float height, Action<Texture2D> callback) {

            AndroidTextureCreator.Instance.CreateTexture(width, height, callback);
        }

        public void CreateMaterial(Action<Material> callback) {

            CreateTexture(1, 1, texture => {
                var materialName = "AndroidViewportMaterial";
                #if UNITY_2017_2_OR_NEWER
                    var singlePassStereoRenderingIsEnabled = XRSettings.enabled && XRSettings.eyeTextureDesc.vrUsage == VRTextureUsage.TwoEyes;
                    if (singlePassStereoRenderingIsEnabled) {
                        materialName = "AndroidSinglePassViewportMaterial";
                    }
                #endif // UNITY_2017_2_OR_NEWER
                var material = Resources.Load<Material>(materialName);
                _enableGammaCorrectionIfNeeded(material);
                material.mainTexture = texture;
                callback(material);
            });
        }

        public void CreateVideoMaterial(Action<Material> callback) {

            CreateTexture(1, 1, texture => {
                var materialName = "AndroidVideoMaterial";
                #if UNITY_2017_2_OR_NEWER
                    var singlePassStereoRenderingIsEnabled = XRSettings.enabled && XRSettings.eyeTextureDesc.vrUsage == VRTextureUsage.TwoEyes;
                    if (singlePassStereoRenderingIsEnabled) {
                        materialName = "AndroidSinglePassVideoMaterial";
                    }
                #endif // UNITY_2017_2_OR_NEWER
                var material = Resources.Load<Material>(materialName);
                _enableGammaCorrectionIfNeeded(material);
                material.mainTexture = texture;
                callback(material);
            });
        }

        public virtual IWebView CreateWebView() {

            return AndroidWebView.Instantiate();
        }

        public void SetStorageEnabled(bool enabled) {

            AndroidWebView.SetStorageEnabled(enabled);
        }

        public void SetUserAgent(bool mobile) {

            AndroidWebView.GloballySetUserAgent(mobile);
        }

        public void SetUserAgent(string userAgent) {

            AndroidWebView.GloballySetUserAgent(userAgent);
        }

        static AndroidWebPlugin _instance;

        static void _enableGammaCorrectionIfNeeded(Material material) {
            // When linear color space is enabled, the shader must apply gamma correction
            // manually in order to prevent the texture from appearing bright and washed
            // out due to incorrect gamma.
            if (QualitySettings.activeColorSpace == ColorSpace.Linear) {
                material.SetFloat("_EnableGammaCorrection", 1.0f);
            }
        }

        /**
        * Some platforms, like Oculus, require that processing and
        * media playback be paused when the app is paused, so this
        * takes care of pausing and resuming processing.
        */
        void OnApplicationPause(bool isPaused) {

            if (isPaused) {
                AndroidWebView.PauseAll();
            } else {
                AndroidWebView.ResumeAll();
            }
        }
    }
}
#endif // UNITY_ANDROID && !UNITY_EDITOR
