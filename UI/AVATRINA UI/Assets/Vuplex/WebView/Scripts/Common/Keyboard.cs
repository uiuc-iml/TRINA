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
    /// A 3D, on-screen keyboard that you can programmatically place in your scene.
    /// </summary>
    /// <remarks>
    /// The keyboard is a web UI that runs inside a `WebViewPrefab` instance and
    /// emits messages to the C# to indicate when characters have been pressed.
    /// [The keyboard UI is open source and available on GitHub](https://github.com/vuplex/unity-keyboard).
    /// </remarks>
    /// <example>
    /// Example:
    /// ```
    /// // First, create a WebViewPrefab for our main web content.
    /// var mainWebContent = WebViewPrefab.Instantiate(0.6f, 0.3f);
    /// mainWebContent.transform.parent = transform;
    /// mainWebContent.transform.localPosition = new Vector3(0, 0f, 0.4f);
    /// mainWebContent.transform.LookAt(transform);
    /// mainWebContent.Initialized += (sender, e) => mainWebContent.WebView.LoadUrl("https://www.google.com");
    ///
    /// // Add the keyboard under the main webview.
    /// var keyboard = Keyboard.Instantiate();
    /// keyboard.transform.parent = mainWebContent.transform;
    /// keyboard.transform.localPosition = new Vector3(0, -0.31f, 0);
    /// keyboard.transform.localEulerAngles = new Vector3(0, 0, 0);
    /// // Hook up the keyboard so that characters are routed to the main webview.
    /// keyboard.InputReceived += (sender, e) => mainWebContent.WebView.HandleKeyboardInput(e.Value);
    /// ```
    /// </example>
    /// <remarks>
    /// The keyboard supports layouts for the following languages and automatically sets the layout
    /// based on the operating system's default language:
    /// - English
    /// - Spanish
    /// - French
    /// - German
    /// - Russian
    /// - Danish
    /// - Norwegian
    /// - Swedish
    /// </remarks>
    public class Keyboard : MonoBehaviour {
        /// <summary>
        /// Indicates that the user clicked a key on the keyboard.
        /// </summary>
        public event EventHandler<EventArgs<string>> InputReceived;

        /// <summary>
        /// Indicates that the keyboard finished initializing.
        /// </summary>
        public event EventHandler Initialized;

        /// <summary>
        /// The `WebViewPrefab` instance used for the keyboard UI.
        /// </summary>
        public WebViewPrefab WebViewPrefab { get; private set; }

        /// <summary>
        /// Creates and initializes a `Keyboard` instance using the default width and height.
        /// </summary>
        public static Keyboard Instantiate() {

            return Instantiate(DEFAULT_KEYBOARD_WIDTH, DEFAULT_KEYBOARD_HEIGHT);
        }

        /// <summary>
        /// Creates and initializes a `Keyboard` instance using the specified width and height.
        /// </summary>
        public static Keyboard Instantiate(float width, float height) {

            var keyboard = (Keyboard) new GameObject("Keyboard").AddComponent<Keyboard>();
            keyboard.Init(width, height);
            return keyboard;
        }

        /// <summary>
        /// Initializes the keyboard with the specified width and height.
        /// </summary>
        /// <remarks>
        /// `Instantiate()` calls this method for you, so you only need to
        /// call this method directly if you place the `Keyboard.cs` script
        /// on your own custom object.
        /// </remarks>
        public void Init(float width, float height) {

            this.WebViewPrefab = WebViewPrefab.Instantiate(width, height, new WebViewOptions { disableVideo = true });
            this.WebViewPrefab.transform.parent = transform;
            this.WebViewPrefab.transform.localPosition = new Vector3(0, 0, 0);
            this.WebViewPrefab.Initialized += (sender, e) => {
                this.WebViewPrefab.WebView.MessageEmitted += WebView_MessageEmitted;
                this.WebViewPrefab.WebView.LoadHtml(KeyboardUi.Html);
            };
        }

        const float DEFAULT_KEYBOARD_WIDTH = 0.5f;
        const float DEFAULT_KEYBOARD_HEIGHT = 0.125f;

        void WebView_MessageEmitted(object sender, EventArgs<string> e) {

            var serializedMessage = e.Value;
            var messageType = JsonUtility.FromJson<BridgeMessage>(serializedMessage).type;
            switch (messageType) {
                case "keyboard.inputReceived":
                    var input = StringBridgeMessage.ParseValue(serializedMessage);
                    if (InputReceived != null) {
                        InputReceived(this, new EventArgs<string>(input));
                    }
                    break;
                case "keyboard.initialized":
                    _sendKeyboardLanguageMessage();
                    if (Initialized != null) {
                        Initialized(this, EventArgs.Empty);
                    }
                    break;
            }
        }

        string _getKeyboardLanguage() {
            switch (Application.systemLanguage) {
                case SystemLanguage.Danish:
                    return "da";
                case SystemLanguage.French:
                    return "fr";
                case SystemLanguage.German:
                    return "de";
                case SystemLanguage.Norwegian:
                    return "no";
                case SystemLanguage.Russian:
                    return "ru";
                case SystemLanguage.Spanish:
                    return "es";
                case SystemLanguage.Swedish:
                    return "sv";
                default:
                    return "en";
            }
        }

        /**
        * Initializes the keyboard language based on the system language.
        */
        void _sendKeyboardLanguageMessage() {

            var message = new StringBridgeMessage {
                type = "keyboard.setLanguage",
                value = _getKeyboardLanguage()
            };
            var serializedMessage = JsonUtility.ToJson(message);
            this.WebViewPrefab.WebView.PostMessage(serializedMessage);
        }
    }
}
