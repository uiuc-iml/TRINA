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

#if UNITY_2017_2_OR_NEWER
    using UnityEngine.XR;
#else
    using XRSettings = UnityEngine.VR.VRSettings;
#endif // UNITY_2017_2_OR_NEWER

namespace Vuplex.WebView {

    /// <summary>
    /// Sets up the WebViewDemo scene, which displays a webview
    /// with an on-screen keyboard underneath it.
    /// </summary>
    /// <remarks>
    /// This scene includes Unity's standalone input module, so
    /// you can click and scroll the webview using your touchscreen
    /// or mouse.
    ///
    /// You can also move the camera by holding down the control key on your
    /// keyboard and moving your mouse. When running on a device
    /// with a gyroscope, the gyroscope controls the camera rotation instead.
    ///
    /// `WebViewPrefab` handles standard Unity input events, so it works with
    /// a variety of third party input modules that extend Unity's BaseInputModule,
    /// like the input modules from the Google VR and Oculus VR SDKs.
    ///
    /// Here are some other examples that show how to use 3D WebView with popular SDKs:
    /// • Google VR (Cardboard and Daydream): https://github.com/vuplex/google-vr-webview-example
    /// • Oculus (Oculus Quest, Go, and Gear VR): https://github.com/vuplex/oculus-webview-example
    /// • AR Foundation : https://github.com/vuplex/ar-foundation-webview-example
    /// </remarks>
    class WebViewDemoSceneManager : MonoBehaviour {

        public GameObject InstructionMessage;

        Vector2 _rotationFromMouse;
        WebViewPrefab _webViewPrefab;

        void Start() {

            // Create a 0.6 x 0.3 instance of the prefab.
            _webViewPrefab = WebViewPrefab.Instantiate(0.6f, 0.3f);
            _webViewPrefab.transform.parent = transform;
            _webViewPrefab.transform.localPosition = new Vector3(0, 0f, 0.4f);
            _webViewPrefab.transform.LookAt(transform);
            _webViewPrefab.Initialized += (sender, e) => {
                _webViewPrefab.WebView.LoadUrl("http://130.126.138.139:8888/");
            };

            // Add an on-screen keyboard under the main webview.
            var keyboard = Keyboard.Instantiate();
            keyboard.transform.parent = _webViewPrefab.transform;
            keyboard.transform.localPosition = new Vector3(0, -0.31f, 0);
            keyboard.transform.localEulerAngles = new Vector3(0, 0, 0);
            // Hook up the on-screen keyboard so that characters are routed to webview.
            keyboard.InputReceived += (sender, e) => {
                _webViewPrefab.WebView.HandleKeyboardInput(e.Value);
            };

            // If VR is disabled, enable the gyro so that it can be used to control the camera rotation.
            if (!XRSettings.enabled) {
                Input.gyro.enabled = true;
            }

            // Show the instruction tip in the editor.
            if (Application.isEditor && InstructionMessage != null) {
                InstructionMessage.SetActive(true);
            } else {
                InstructionMessage = null;
            }
        }

        void Update() {

            _updateCameraRotation();
            _handleHardwareKeyboardInput();

            // Dismiss the instruction message on the first click.
            if (InstructionMessage != null && Input.GetMouseButtonDown(0)) {
                InstructionMessage.SetActive(false);
                InstructionMessage = null;
            }
        }

        /// <summary>
        /// Enables typing via the hardware keyboard by
        /// passing characters from `Input.inputString` to the webview.
        /// </summary>
        void _handleHardwareKeyboardInput() {

            foreach (var character in Input.inputString) {
                string characterString;
                switch (character) {
                    case '\b':
                        characterString = "Backspace";
                        break;
                    case '\n':
                        characterString = "Enter";
                        break;
                    default:
                        characterString = character.ToString();
                        break;
                }
                _webViewPrefab.WebView.HandleKeyboardInput(characterString);
            }
            if (Input.GetKeyDown(KeyCode.UpArrow)) {
                _webViewPrefab.WebView.HandleKeyboardInput("ArrowUp");
            }
            if (Input.GetKeyDown(KeyCode.DownArrow)) {
                _webViewPrefab.WebView.HandleKeyboardInput("ArrowDown");
            }
            if (Input.GetKeyDown(KeyCode.RightArrow)) {
                _webViewPrefab.WebView.HandleKeyboardInput("ArrowRight");
            }
            if (Input.GetKeyDown(KeyCode.LeftArrow)) {
                _webViewPrefab.WebView.HandleKeyboardInput("ArrowLeft");
            }
        }

        /// <summary>
        /// If the device has a gyroscope, it is used to control the camera
        /// rotation. Otherwise, the user can hold down the control key on
        /// the keyboard to make the mouse control camera rotation.
        /// </summary>
        void _updateCameraRotation() {

            if (XRSettings.enabled) {
                // VR is enabled, so let the VR SDK control camera rotation instead.
                return;
            }

            if (SystemInfo.supportsGyroscope) {
                Camera.main.transform.Rotate(
                    -Input.gyro.rotationRateUnbiased.x,
                    -Input.gyro.rotationRateUnbiased.y,
                    Input.gyro.rotationRateUnbiased.z
                );
            } else if (Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)) {
                float sensitivity = 10f;
                float maxYAngle = 80f;
                _rotationFromMouse.x += Input.GetAxis("Mouse X") * sensitivity;
                _rotationFromMouse.y -= Input.GetAxis("Mouse Y") * sensitivity;
                _rotationFromMouse.x = Mathf.Repeat(_rotationFromMouse.x, 360);
                _rotationFromMouse.y = Mathf.Clamp(_rotationFromMouse.y, -maxYAngle, maxYAngle);
                Camera.main.transform.rotation = Quaternion.Euler(_rotationFromMouse.y, _rotationFromMouse.x, 0);
            }
        }
    }
}
