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
using UnityEngine;

namespace Vuplex.WebView {

    /// <summary>
    /// Script that helps with setting the video-related shader properties on mobile.
    /// </summary>
    public class ViewportMaterialView : MonoBehaviour {

        /// <summary>
        /// The view's texture, which is `null` until the material has been set.
        /// </summary>
        public Texture2D Texture {
            get {
                return (Texture2D) GetComponent<Renderer>().material.mainTexture;
            }
            set {
                GetComponent<Renderer>().material.mainTexture = value;
            }
        }

        public void SetCropRect(Rect rect) {

            GetComponent<Renderer>().material.SetVector("_CropRect", _rectToVector(rect));
        }

        public void SetCutoutRect(Rect rect) {

            var rectVector = _rectToVector(rect);
            // Make the actual cutout slightly smaller (2% shorter and 2% skinnier) so that
            // the gap between the video layer and the viewport isn't visible.
            var onePercentOfWidth = rect.width * 0.01f;
            var onePercentOfHeight = rect.height * 0.01f;
            var slightlySmallerRect = new Vector4(
                rectVector.x + onePercentOfWidth,
                rectVector.y + onePercentOfHeight,
                rectVector.z - 2 * onePercentOfWidth,
                rectVector.w - 2 * onePercentOfHeight
            );
            GetComponent<Renderer>().material.SetVector("_VideoCutoutRect", slightlySmallerRect);
        }

        public void SetMaterial(Material material) {

            GetComponent<Renderer>().material = material;
        }

        public void SetStereoToMonoOverride(bool overrideStereoToMono) {

            GetComponent<Renderer>().material.SetFloat("_OverrideStereoToMono", overrideStereoToMono ? 1.0f : 0);
        }

        Vector4 _rectToVector(Rect rect) {

            return new Vector4(rect.x, rect.y, rect.width, rect.height);
        }
    }
}

