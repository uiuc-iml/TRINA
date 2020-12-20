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
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UnityEngine;

#if NET_4_6
using System.Threading.Tasks;
#endif // NET_4_6

namespace Vuplex.WebView {
    /// <summary>
    /// The base `IWebView` implementation, which is extended for each platform.
    /// </summary>
    public abstract class BaseWebView : MonoBehaviour {

        public event EventHandler<ProgressChangedEventArgs> LoadProgressChanged;

        public event EventHandler<EventArgs<string>> MessageEmitted  {
            add { _messageHandler.MessageEmitted += value; }
            remove { _messageHandler.MessageEmitted -= value; }
        }

        public event EventHandler PageLoadFailed;

        public event EventHandler<EventArgs<string>> TitleChanged  {
            add { _messageHandler.TitleChanged += value; }
            remove { _messageHandler.TitleChanged -= value; }
        }

        public event EventHandler<UrlChangedEventArgs> UrlChanged;

        public event EventHandler<EventArgs<Rect>> VideoRectChanged  {
            add { _messageHandler.VideoRectChanged += value; }
            remove { _messageHandler.VideoRectChanged -= value; }
        }

        public int Resolution {
            get {
                return _numberOfPixelsPerUnityUnit;
            }
        }

        public Vector2 Size {
            get {
                return new Vector2(_width, _height);
            }
        }

        public Vector2 SizeInPixels {
            get {
                return new Vector2(_nativeWidth, _nativeHeight);
            }
        }

        public Texture2D Texture {
            get {
                return _viewportTexture;
            }
        }

        public string Url { get; private set; }

        public Texture2D VideoTexture {
            get {
                return _viewportTexture;
            }
        }

        public void Init(Texture2D texture, float width, float height) {

            Init(texture, width, height, null);
        }

        public virtual void Init(Texture2D viewportTexture, float width, float height, Texture2D videoTexture) {

            _viewportTexture = viewportTexture;
            _videoTexture = videoTexture;
            // Assign the game object a unique name so that the native view can send it messages.
            gameObject.name = String.Format("WebView-{0}", Guid.NewGuid().ToString());
            _width = width;
            _height = height;
            _messageHandler.UrlChanged += MessageHandler_UrlChanged;
        }

        public virtual void Blur() {

            WebView_blur(_nativeWebViewPtr);
        }

    #if NET_4_6
        public Task<bool> CanGoBack() {

            var task = new TaskCompletionSource<bool>();
            CanGoBack(task.SetResult);
            return task.Task;
        }

        public Task<bool> CanGoForward() {

            var task = new TaskCompletionSource<bool>();
            CanGoForward(task.SetResult);
            return task.Task;
        }
    #endif // NET_4_6

        public abstract void CanGoBack(Action<bool> callback);

        public abstract void CanGoForward(Action<bool> callback);

        public virtual void Click(Vector2 point) {

            int nativeX = (int) (point.x * _nativeWidth);
            int nativeY = (int) (point.y * _nativeHeight);
            WebView_click(_nativeWebViewPtr, nativeX, nativeY);
        }

    #if NET_4_6
        public Task<byte[]> CaptureScreenshot() {

            var task = new TaskCompletionSource<byte[]>();
            CaptureScreenshot(task.SetResult);
            return task.Task;
        }
    #endif // NET_4_6

        public abstract void CaptureScreenshot(Action<byte[]> callback);

        public static void CreateTexture(float width, float height, Action<Texture2D> callback) {

            int nativeWidth = (int)(width * Config.NumberOfPixelsPerUnityUnit);
            int nativeHeight = (int)(height * Config.NumberOfPixelsPerUnityUnit);
            var nativeTexture = WebView_createTexture(nativeWidth, nativeHeight, SystemInfo.graphicsDeviceType.ToString());
            Texture2D texture = Texture2D.CreateExternalTexture(
                nativeWidth,
                nativeHeight,
                TextureFormat.RGBA32,
                false,
                false,
                nativeTexture
            );
            // Invoke the callback asynchronously in order to match the async
            // behavior that's required for Android.
            Dispatcher.RunOnMainThread(() => callback(texture));
        }

        public virtual void DisableViewUpdates() {

            WebView_disableViewUpdates(_nativeWebViewPtr);
            _viewUpdatesAreEnabled = false;
        }

        public virtual void Dispose() {

            _isDisposed = true;
            // Unity destroys all of the GameObjects after OnApplicationQuit().
            // In that scenario, the native resources have already been destroyed
            // by the call to WebView_terminatePlugin().
            if (!_pluginTerminated) {
                WebView_destroy(_nativeWebViewPtr);
            }
            _nativeWebViewPtr = IntPtr.Zero;
            Destroy(gameObject);
        }

        public virtual void EnableViewUpdates() {

            if (_currentViewportNativeTexture != IntPtr.Zero) {
                _viewportTexture.UpdateExternalTexture(_currentViewportNativeTexture);
            }
            // After view updates are enabled, the native webview will
            // emit a new native texture (HandleTextureChanged)
            // if it was resized since it was last active.
            WebView_enableViewUpdates(_nativeWebViewPtr);
            _viewUpdatesAreEnabled = true;
        }

    #if NET_4_6
        public Task<string> ExecuteJavaScript(string javaScript) {

            var task = new TaskCompletionSource<string>();
            ExecuteJavaScript(javaScript, task.SetResult);
            return task.Task;
        }
    #else
        public void ExecuteJavaScript(string javaScript) {

            ExecuteJavaScript(javaScript, null);
        }
    #endif // NET_4_6

        public virtual void ExecuteJavaScript(string javaScript, Action<string> callback) {

            if (_isDisposed) {
                if (callback != null) {
                    callback("");
                }
            } else {
                string resultCallbackId = null;
                if (callback != null) {
                    resultCallbackId = Guid.NewGuid().ToString();
                    _pendingJavaScriptResultCallbacks[resultCallbackId] = callback;
                }
                WebView_executeJavaScript(_nativeWebViewPtr, javaScript, resultCallbackId);
            }
        }

        public virtual void Focus() {

            WebView_focus(_nativeWebViewPtr);
        }

    #if NET_4_6
        public Task<byte[]> GetRawTextureData() {

            var task = new TaskCompletionSource<byte[]>();
            GetRawTextureData(task.SetResult);
            return task.Task;
        }
    #endif // NET_4_6

        public virtual void GetRawTextureData(Action<byte[]> callback) {

            IntPtr unmanagedBytes = IntPtr.Zero;
            int unmanagedBytesLength = 0;
            WebView_getRawTextureData(_nativeWebViewPtr, ref unmanagedBytes, ref unmanagedBytesLength);

            // Load the results into a managed array.
            var managedBytes = new byte[unmanagedBytesLength];
            Marshal.Copy(unmanagedBytes, managedBytes, 0, unmanagedBytesLength);
            WebView_freeMemory(unmanagedBytes);
            callback(managedBytes);
        }

        public static void GloballySetUserAgent(bool mobile) {

            var success = WebView_globallySetUserAgentToMobile(mobile);
            if (!success) {
                throw new InvalidOperationException(USER_AGENT_EXCEPTION_MESSAGE);
            }
        }

        public static void GloballySetUserAgent(string userAgent) {

            var success = WebView_globallySetUserAgent(userAgent);
            if (!success) {
                throw new InvalidOperationException(USER_AGENT_EXCEPTION_MESSAGE);
            }
        }

        public virtual void GoBack() {

            WebView_goBack(_nativeWebViewPtr);
        }

        public virtual void GoForward() {

            WebView_goForward(_nativeWebViewPtr);
        }

        public virtual void HandleKeyboardInput(string input) {

            WebView_handleKeyboardInput(_nativeWebViewPtr, input);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleLoadFailed(string unusedParam) {

            if (PageLoadFailed != null) {
                PageLoadFailed(this, EventArgs.Empty);
            }
            var e = new ProgressChangedEventArgs(ProgressChangeType.Failed, 1.0f);
            OnLoadProgressChanged(e);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleLoadFinished(string unusedParam) {

            var e = new ProgressChangedEventArgs(ProgressChangeType.Finished, 1.0f);
            OnLoadProgressChanged(e);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleLoadStarted(string unusedParam) {

            var e = new ProgressChangedEventArgs(ProgressChangeType.Started, 0.0f);
            OnLoadProgressChanged(e);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleLoadProgressUpdate(string progressString) {

            var progress = float.Parse(progressString);
            var e = new ProgressChangedEventArgs(ProgressChangeType.Updated, progress);
            OnLoadProgressChanged(e);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleMessageEmitted(string serializedMessage) {

            _messageHandler.HandleMessage(serializedMessage);
        }

        /// <summary>
        /// The native plugin invokes this method.
        /// </summary>
        public void HandleTextureChanged(string textureString) {

            var nativeTexture = new IntPtr(Int64.Parse(textureString));
            if (nativeTexture == _currentViewportNativeTexture) {
                return;
            }
            var previousNativeTexture = _currentViewportNativeTexture;
            _currentViewportNativeTexture = nativeTexture;
            if (_viewUpdatesAreEnabled) {
                _viewportTexture.UpdateExternalTexture(_currentViewportNativeTexture);
            }

            if (previousNativeTexture != IntPtr.Zero && previousNativeTexture != _currentViewportNativeTexture) {
                WebView_destroyTexture(previousNativeTexture, SystemInfo.graphicsDeviceType.ToString());
            }
        }

        public virtual void LoadHtml(string html) {

            WebView_loadHtml(_nativeWebViewPtr, html);
        }

        public virtual void LoadUrl(string url) {

            WebView_loadUrl(_nativeWebViewPtr, url);
        }

        public virtual void LoadUrl(string url, Dictionary<string, string> additionalHttpHeaders) {

            if (additionalHttpHeaders == null) {
                LoadUrl(url);
            } else {
                var headerStrings = additionalHttpHeaders.Keys.Select(key => String.Format("{0}: {1}", key, additionalHttpHeaders[key])).ToArray();
                var newlineDelimitedHttpHeaders = String.Join("\n", headerStrings);
                WebView_loadUrlWithHeaders(_nativeWebViewPtr, url, newlineDelimitedHttpHeaders);
            }
        }

        public void PostMessage(string data) {

            var escapedString = data.Replace("'", "\\'");
            ExecuteJavaScript(String.Format("vuplex._emit('message', {{ data: \'{0}\' }})", escapedString));
        }

        public virtual void Reload() {

            WebView_reload(_nativeWebViewPtr);
        }

        public void Resize(float width, float height) {

            if (_isDisposed || (width == _width && height == _height)) {
                return;
            }
            _width = width;
            _height = height;
            _resize();
        }

        public virtual void Scroll(Vector2 delta) {

            var deltaX = (int)(delta.x * _numberOfPixelsPerUnityUnit);
            var deltaY = (int)(delta.y * _numberOfPixelsPerUnityUnit);
            WebView_scroll(_nativeWebViewPtr, deltaX, deltaY);
        }

        public virtual void Scroll(Vector2 scrollDelta, Vector2 mousePosition) {

            var deltaX = (int)(scrollDelta.x * _numberOfPixelsPerUnityUnit);
            var deltaY = (int)(scrollDelta.y * _numberOfPixelsPerUnityUnit);
            var mouseX = (int)(mousePosition.x * _nativeWidth);
            var mouseY = (int)(mousePosition.y * _nativeHeight);
            WebView_scrollAtPoint(_nativeWebViewPtr, deltaX, deltaY, mouseX, mouseY);
        }

        public void SetResolution(int pixelsPerUnityUnit) {

            _numberOfPixelsPerUnityUnit = pixelsPerUnityUnit;
            _resize();
        }

        public virtual void ZoomIn() {

            WebView_zoomIn(_nativeWebViewPtr);
        }

        public virtual void ZoomOut() {

            WebView_zoomOut(_nativeWebViewPtr);
        }

        protected IntPtr _currentViewportNativeTexture;

    #if UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
        protected const string _dllName = "VuplexWebViewWindows";
    #elif UNITY_STANDALONE_OSX || UNITY_EDITOR_OSX
        protected const string _dllName = "VuplexWebViewMac";
    #elif UNITY_ANDROID
        protected const string _dllName = "VuplexWebViewAndroid";
    #else
        protected const string _dllName = "__Internal";
    #endif

        protected float _height; // in Unity units
        protected bool _isDisposed;
        protected BridgeMessageHandler _messageHandler = new BridgeMessageHandler();
        // Height in pixels.
        protected int _nativeHeight { get { return (int)(_height * _numberOfPixelsPerUnityUnit); }}
        protected IntPtr _nativeWebViewPtr = IntPtr.Zero;
        // Width in pixels.
        protected int _nativeWidth { get { return (int)(_width * _numberOfPixelsPerUnityUnit); }}
        protected int _numberOfPixelsPerUnityUnit = Config.NumberOfPixelsPerUnityUnit;
        protected Dictionary<string, Action<string>> _pendingJavaScriptResultCallbacks = new Dictionary<string, Action<string>>();
        protected static bool _pluginTerminated;
        const string USER_AGENT_EXCEPTION_MESSAGE = "Unable to set the User-Agent string, because a webview has already been created with the default User-Agent. On Windows and macOS, SetUserAgent() can only be called prior to creating any webviews.";
        protected Texture2D _videoTexture;
        protected bool _viewUpdatesAreEnabled = true;
        protected Texture2D _viewportTexture;
        protected float _width; // in Unity units

        protected void MessageHandler_UrlChanged(object sender, UrlChangedEventArgs e) {

            OnUrlChanged(e);
        }

        protected virtual void OnLoadProgressChanged(ProgressChangedEventArgs e) {

            if (LoadProgressChanged != null) {
                LoadProgressChanged(this, e);
            }
        }

        protected virtual void OnUrlChanged(UrlChangedEventArgs e) {

            if (Url == e.Url) {
                return;
            }
            Url = e.Url;
            if (UrlChanged != null) {
                UrlChanged(this, e);
            }
        }

        protected virtual void _resize() {

            // Only trigger a resize if the webview has been initialized
            if (_viewportTexture) {
                WebView_resize(_nativeWebViewPtr, _nativeWidth, _nativeHeight);
            }
        }

        static void _unitySendMessage(string gameObjectName, string methodName, string message) {

            Dispatcher.RunOnMainThread(() => {
                var gameObj = GameObject.Find(gameObjectName);
                if (gameObj == null) {
                    Debug.LogErrorFormat("Unable to send the message, because there is no GameObject named '{0}'", gameObjectName);
                    return;
                }
                gameObj.SendMessage(methodName, message);
            });
        }

        [DllImport(_dllName)]
        static extern void WebView_blur(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_click(IntPtr webViewPtr, int x, int y);

        [DllImport(_dllName)]
        protected static extern IntPtr WebView_createTexture(int width, int height, string graphicsApi);

        [DllImport(_dllName)]
        protected static extern void WebView_destroyTexture(IntPtr texture, string graphicsApi);

        [DllImport(_dllName)]
        static extern void WebView_destroy(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_disableViewUpdates(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_enableViewUpdates(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_executeJavaScript(IntPtr webViewPtr, string javaScript, string resultCallbackId);

        [DllImport(_dllName)]
        static extern void WebView_focus(IntPtr webViewPtr);

        [DllImport(_dllName)]
        protected static extern void WebView_freeMemory(IntPtr bytes);

        [DllImport(_dllName)]
        static extern void WebView_getRawTextureData(IntPtr webViewPtr, ref IntPtr bytes, ref int length);

        [DllImport(_dllName)]
        static extern bool WebView_globallySetUserAgentToMobile(bool mobile);

        [DllImport(_dllName)]
        static extern bool WebView_globallySetUserAgent(string userAgent);

        [DllImport(_dllName)]
        static extern void WebView_goBack(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_goForward(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_handleKeyboardInput(IntPtr webViewPtr, string input);

        [DllImport(_dllName)]
        static extern void WebView_loadHtml(IntPtr webViewPtr, string html);

        [DllImport(_dllName)]
        static extern void WebView_loadUrl(IntPtr webViewPtr, string url);

        [DllImport(_dllName)]
        static extern void WebView_loadUrlWithHeaders(IntPtr webViewPtr, string url, string newlineDelimitedHttpHeaders);

        [DllImport(_dllName)]
        static extern void WebView_reload(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_resize(IntPtr webViewPtr, int width, int height);

        [DllImport(_dllName)]
        static extern void WebView_scroll(IntPtr webViewPtr, int deltaX, int deltaY);

        [DllImport(_dllName)]
        static extern void WebView_scrollAtPoint(IntPtr webViewPtr, int deltaX, int deltaY, int mouseX, int mouseY);

        [DllImport(_dllName)]
        static extern void WebView_zoomIn(IntPtr webViewPtr);

        [DllImport(_dllName)]
        static extern void WebView_zoomOut(IntPtr webViewPtr);
    }
}
