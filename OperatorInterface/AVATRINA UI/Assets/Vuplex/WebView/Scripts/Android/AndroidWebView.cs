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
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Vuplex.WebView {

    /// <summary>
    /// The Android `IWebView` implementation, which also includes
    /// extra methods for Android-specific functionality.
    /// </summary>
    public class AndroidWebView : BaseWebView, IWebView {

        public static AndroidWebView Instantiate() {

            return (AndroidWebView) new GameObject().AddComponent<AndroidWebView>();
        }

        public override void Init(Texture2D viewportTexture, float width, float height, Texture2D videoTexture) {

            AssertWebViewIsAvailable();
            base.Init(viewportTexture, width, height, videoTexture);
            _webView = new AndroidJavaObject(
                FULL_CLASS_NAME,
                gameObject.name,
                viewportTexture.GetNativeTexturePtr().ToInt32(),
                _nativeWidth,
                _nativeHeight,
                SystemInfo.graphicsMultiThreaded,
                videoTexture != null
            );
        }

        internal static void AssertWebViewIsAvailable() {

            if (!IsWebViewAvailable()) {
                throw new WebViewUnavailableException("The Android WebView package is currently unavailable. This is rare but can occur if it's not installed on the system or is currently being updated.");
            }
        }

        public override void Blur() {

            if (!_isDisposed) {
                _webView.Call("blur");
            }
        }

        public override void CanGoBack(Action<bool> callback) {

            _webView.Call("canGoBack", new AndroidBoolCallback(callback));
        }

        public override void CanGoForward(Action<bool> callback) {

            _webView.Call("canGoForward", new AndroidBoolCallback(callback));
        }

        public override void CaptureScreenshot(Action<byte[]> callback) {

            if (_isDisposed) {
                callback(new byte[0]);
            } else {
                _webView.Call("captureScreenshot", new AndroidByteArrayCallback(callback));
            }
        }

        public static void ClearAllData() {

            _class.CallStatic("clearAllData");
        }

        /// <summary>
        /// Clears the webview's back / forward navigation history.
        /// </summary>
        public void ClearHistory() {

            _webView.Call("clearHistory");
        }

        public override void Click(Vector2 point) {

            if (_isDisposed) {
                return;
            }
            var nativeX = (int)(point.x * _nativeWidth);
            var nativeY = (int)(point.y * _nativeHeight);
            _webView.Call("click", nativeX, nativeY);
        }

        public override void DisableViewUpdates() {

            _webView.Call("disableViewUpdates");
            _viewUpdatesAreEnabled = false;
        }

        public override void Dispose() {

            if (_isDisposed) {
                return;
            }
            // Cancel the render if it has been scheduled via GL.IssuePluginEvent().
            WebView_removePointer(_webView.GetRawObject());
            _isDisposed = true;
            _webView.Call("destroy");
            _webView.Dispose();
            Destroy(gameObject);
        }

        public override void EnableViewUpdates() {

            _webView.Call("enableViewUpdates");
            _viewUpdatesAreEnabled = true;
        }

        public override void ExecuteJavaScript(string javaScript, Action<string> callback) {

            if (_isDisposed) {
                if (callback != null) {
                    callback("");
                }
            } else {
                _webView.Call("executeJavaScript", javaScript, new AndroidStringCallback(callback));
            }
        }

        public override void Focus() {

            if (!_isDisposed) {
                _webView.Call("focus");
            }
        }

        public override void GetRawTextureData(Action<byte[]> callback) {

            if (_isDisposed) {
                callback(new byte[0]);
            } else {
                _webView.Call("getRawTextureData", new AndroidByteArrayCallback(callback));
            }
        }

        public static void GloballySetUserAgent(bool mobile) {

            _class.CallStatic("globallySetUserAgent", mobile);
        }

        public static void GloballySetUserAgent(string userAgent) {

            _class.CallStatic("globallySetUserAgent", userAgent);
        }

        /// <summary>
        /// Some standalone VR systems (like the Mirage Solo) use a version of
        /// Android where the primary system used for dispatching mouse and keyboard events
        /// stops working after a while. If you're running on one of those platforms, you
        /// can call this method to make webviews use an alternative input event
        /// system that doesn't stop working after a while. The tradeoff is that this
        /// alternative input event system doesn't handle mouse and keyboard events
        /// as accurately as the primary method does (e.g. scrolling in some elements may
        /// not work as expected). Note that this is no longer needed for Oculus Go.
        /// </summary>
        public static void GloballyUseAlternativeInputEventSystem(bool useAlternativeInputEventSystem) {

            _class.CallStatic("globallyUseAlternativeInputEventSystem", useAlternativeInputEventSystem);
        }

        public override void GoBack() {

            _webView.Call("goBack");
        }

        public override void GoForward() {

            _webView.Call("goForward");
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public virtual void HandleInitialVideoPlayRequest(string serializedVideo) {

            if (_isDisposed) {
                return;
            }
            var video = JsonUtility.FromJson<Video>(serializedVideo);
            var nativeVideoPlayer = _webView.Call<AndroidJavaObject>("getOrCreateVideoPlayer", serializedVideo, _videoTexture.GetNativeTexturePtr().ToInt32());
            nativeVideoPlayer.Call("play", video.videoUrl);
        }

        public override void HandleKeyboardInput(string input) {

            if (!_isDisposed) {
                _webView.Call("handleKeyboardInput", input);
            }
        }

        /// <summary>
        /// Indicates whether the Android WebView package is installed on the system and available.
        /// </summary>
        /// <remarks>
        /// 3D WebView internally depends on Android's WebView package, which is normally installed
        /// as part of the operating system. In rare circumstances, the Android WebView package may be unavailable.
        /// For example, this can happen if the user used developer tools to delete the WebView package
        /// or if [updates to the WebView package are currently being installed](https://bugs.chromium.org/p/chromium/issues/detail?id=506369) .
        /// </remarks>
        public static bool IsWebViewAvailable() {

            if (_webViewPackageIsAvailable == null) {
                _webViewPackageIsAvailable = _class.CallStatic<bool>("isWebViewAvailable");
            }
            return (bool)_webViewPackageIsAvailable;
        }

        public override void LoadHtml(string html) {

            if (!_isDisposed) {
                _webView.Call("loadHtml", html);
            }
        }

        /// <summary>
        /// Like `LoadHtml(string html)`, but also allows a virtual base URL
        /// to be specified.
        /// </summary>
        public void LoadHtml(string html, string baseUrl) {

            if (!_isDisposed) {
                _webView.Call("loadHtml", html, baseUrl);
            }
        }

        public override void LoadUrl(string url) {

            if (!_isDisposed) {
                _webView.Call("loadUrl", url);
            }
        }

        public override void LoadUrl(string url, Dictionary<string, string> additionalHttpHeaders) {

            if (_isDisposed) {
                return;
            }
            if (additionalHttpHeaders == null) {
                LoadUrl(url);
            } else {
                var map = _convertDictionaryToJavaMap(additionalHttpHeaders);
                _webView.Call("loadUrl", url, map);
            }
        }

        /// <summary>
        /// Moves the pointer to the given point in the webpage.
        /// This can be used, for example, to trigger hover effects in the page.
        /// </summary>
        /// <param name="point">
        /// The x and y components of the point are values
        /// between 0 and 1 that are normalized to the width and height, respectively. For example,
        /// `point.x = x in Unity units / width in Unity units`.
        /// Like in the browser, the origin is in the upper-left corner,
        /// the positive direction of the y-axis is down, and the positive
        /// direction of the x-axis is right.
        /// </param>
        public void MovePointer(Vector2 point) {

            if (_isDisposed) {
                return;
            }
            var nativeX = (int)(point.x * _nativeWidth);
            var nativeY = (int)(point.y * _nativeHeight);
            _webView.Call("moveMouse", nativeX, nativeY);
        }

        /// <summary>
        /// Pauses processing, media, and rendering for this webview instance
        /// until `Resume()` is called.
        /// </summary>
        public void Pause() {

            _webView.Call("pause");
        }

        /// <summary>
        /// Pauses processing, media, and rendering for all webview instances.
        /// This method is automatically called by the plugin when the application
        /// is paused.
        /// </summary>
        public static void PauseAll() {

            _class.CallStatic("pauseAll");
        }

        public override void Reload() {

            if (!_isDisposed) {
                _webView.Call("reload");
            }
        }

        /// <summary>
        /// Resumes processing and rendering for all webview instances
        /// after a previous call to `Pause().`
        /// </summary>
        public void Resume() {

            _webView.Call("resume");
        }

        /// <summary>
        /// Resumes processing and rendering for all webview instances
        /// after a previous call to `PauseAll().` This method
        /// is automatically called by the plugin when the application resumes after
        /// being paused.
        /// </summary>
        public static void ResumeAll() {

            _class.CallStatic("resumeAll");
        }

        public override void Scroll(Vector2 scrollDelta) {

            var deltaX = (int)(scrollDelta.x * _numberOfPixelsPerUnityUnit);
            var deltaY = (int)(scrollDelta.y * _numberOfPixelsPerUnityUnit);
            _webView.Call("scroll", deltaX, deltaY);
        }

        public override void Scroll(Vector2 scrollDelta, Vector2 mousePosition) {

            var deltaX = (int)(scrollDelta.x * _numberOfPixelsPerUnityUnit);
            var deltaY = (int)(scrollDelta.y * _numberOfPixelsPerUnityUnit);
            var mouseX = (int)(mousePosition.x * _nativeWidth);
            var mouseY = (int)(mousePosition.y * _nativeHeight);
            _webView.Call("scroll", deltaX, deltaY, mouseX, mouseY);
        }

        /// <summary>
        /// By default, web pages cannot access the device's
        /// camera or microphone via JavaScript, even if the user has granted
        /// the app permission to use them. Invoking `SetAudioAndVideoCaptureEnabled(true)` allows
        /// **all web pages** to access the camera and microphone if the user has
        /// granted the app permission to use them via the standard Android permission dialogs.
        /// </summary>
        /// <remarks>
        /// This is useful, for example, to enable WebRTC support.
        /// </remarks>
        public static void SetAudioAndVideoCaptureEnabled(bool enabled) {

            _class.CallStatic("setAudioAndVideoCaptureEnabled", enabled);
        }

        public static void SetClickCorrectionEnabled(bool enabled) {

            _class.CallStatic("setClickCorrectionEnabled", enabled);
        }

        /// <summary>
        /// Sets the initial scale for web content, where 1.0 is the default scale.
        /// </summary>
        public void SetInitialScale(float scale) {

            _webView.Call("setInitialScale", scale);
        }

        /// <summary>
        /// By default, AndroidWebView prevents JavaScript from auto-playing sound
        /// from most sites unless the user has first interacted with the page.
        /// You can call this method to disable or re-enable enforcement of this auto-play policy.
        /// </summary>
        public void SetMediaPlaybackRequiresUserGesture(bool mediaPlaybackRequiresUserGesture) {

            _webView.Call("setMediaPlaybackRequiresUserGesture", mediaPlaybackRequiresUserGesture);
        }

        /// <summary>
        /// Enables or disables the native Android on-screen keyboard for new webviews
        /// (the default is disabled).
        /// </summary>
        public static void SetNativeKeyboardEnabled(bool enabled) {

            _class.CallStatic("setNativeKeyboardEnabled", enabled);
        }

        public static void SetStorageEnabled(bool enabled) {

            _class.CallStatic("setStorageEnabled", enabled);
        }

        /// <summary>
        /// Like `Web.SetUserAgent(bool mobile)`, except it sets the user-agent
        /// for a single webview instance instead of setting it globally.
        /// </summary>
        /// <remarks>
        /// If you globally set a default user-agent using `Web.SetUserAgent()`,
        /// you can still use this method to override the user-agent for a
        /// single webview instance.
        /// </remarks>
        public void SetUserAgent(bool mobile) {

            _webView.Call("setUserAgent", mobile);
        }

        /// <summary>
        /// Like `Web.SetUserAgent(string userAgent)`, except it sets the user-agent
        /// for a single webview instance instead of setting it globally.
        /// </summary>
        /// <remarks>
        /// If you globally set a default user-agent using `Web.SetUserAgent()`,
        /// you can still use this method to override the user-agent for a
        /// single webview instance.
        /// </remarks>
        public void SetUserAgent(string userAgent) {

            _webView.Call("setUserAgent", userAgent);
        }

        /// <summary>
        /// Used to override the setting set by the static `GloballyUseAlternativeInputEventSystem()`
        /// for a single webview.
        /// </summary>
        public void UseAlternativeInputEventSystem(bool useAlternativeInputEventSystem) {

            if (!_isDisposed) {
                _webView.Call("useAlternativeInputEventSystem", useAlternativeInputEventSystem);
            }
        }

        /// <summary>
        /// Zooms in or out by the given factor, which is multiplied by the current zoom level
        /// to reach the new zoom level.
        /// </summary>
        /// <remarks>
        /// Note that the zoom level gets reset when a new page is loaded.
        /// </remarks>
        /// <param name="zoomFactor">
        /// The zoom factor to apply in the range from 0.01 to 100.0.
        /// </param>
        public void ZoomBy(float zoomFactor) {

            _webView.Call("zoomBy", zoomFactor);
        }

        public override void ZoomIn() {

            _webView.Call("zoomIn");
        }

        public override void ZoomOut() {

            _webView.Call("zoomOut");
        }

        internal static AndroidJavaClass _class = new AndroidJavaClass(FULL_CLASS_NAME);
        const string FULL_CLASS_NAME = "com.vuplex.webview.WebView";
        ScreenOrientation _previousOrientation = Screen.orientation;
        internal AndroidJavaObject _webView;
        static bool? _webViewPackageIsAvailable = null;

        AndroidJavaObject _convertDictionaryToJavaMap(Dictionary<string, string> dictionary) {

            AndroidJavaObject map = new AndroidJavaObject("java.util.HashMap");
            IntPtr putMethod = AndroidJNIHelper.GetMethodID(map.GetRawClass(), "put", "(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");
            foreach (var entry in dictionary) {
                AndroidJNI.CallObjectMethod(
                    map.GetRawObject(),
                    putMethod,
                    AndroidJNIHelper.CreateJNIArgArray(new object[] { entry.Key, entry.Value })
                );
            }
            return map;
        }

        IEnumerator _renderPluginOncePerFrame() {
            while (true) {
                // Wait until all frame rendering is done
                yield return new WaitForEndOfFrame();

                if (!_viewUpdatesAreEnabled || _isDisposed) {
                    continue;
                }
                var nativeWebViewPtr = _webView.GetRawObject();
                if (nativeWebViewPtr != IntPtr.Zero) {
                    int pointerId = WebView_depositPointer(nativeWebViewPtr);
                    GL.IssuePluginEvent(WebView_getRenderFunction(), pointerId);
                }
            }
        }

        protected override void _resize() {

            // Only trigger a resize if the webview has been initialized
            if (_viewportTexture) {
                _webView.Call("resize", _nativeWidth, _nativeHeight);
            }
        }

        IEnumerator Start() {

            yield return StartCoroutine(_renderPluginOncePerFrame());
        }

        void Update() {

            if (_previousOrientation != Screen.orientation) {
                _previousOrientation = Screen.orientation;
                _webView.Call("handleScreenOrientationChange");
            }
        }

        [DllImport(_dllName)]
        static extern IntPtr WebView_getRenderFunction();

        [DllImport(_dllName)]
        static extern int WebView_depositPointer(IntPtr pointer);

        [DllImport(_dllName)]
        static extern void WebView_removePointer(IntPtr pointer);
    }
}
#endif // UNITY_ANDROID && !UNITY_EDITOR
