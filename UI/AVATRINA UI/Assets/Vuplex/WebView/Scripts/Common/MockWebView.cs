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
#pragma warning disable CS0067
using System;
using System.Collections.Generic;
using UnityEngine;

#if NET_4_6
using System.Threading.Tasks;
#endif // NET_4_6

namespace Vuplex.WebView {
    /// <summary>
    /// Mock IWebView implementation used for running in the Unity editor.
    /// </summary>
    /// <remarks>
    /// MockWebView logs messages to the console to indicate when its methods are
    /// called, but it doesn't actually load or render web content.
    /// </remarks>
    class MockWebView : IWebView {

        public event EventHandler<ProgressChangedEventArgs> LoadProgressChanged;

        public event EventHandler<EventArgs<string>> MessageEmitted;

        public event EventHandler PageLoadFailed;

        public event EventHandler<EventArgs<string>> TitleChanged;

        public event EventHandler<UrlChangedEventArgs> UrlChanged;

        public event EventHandler<EventArgs<Rect>> VideoRectChanged;

        public int Resolution {
            get {
                return _numberOfPixelsPerUnityUnit;
            }
        }

        public Vector2 Size { get; private set; }

        public Vector2 SizeInPixels {
            get {
                return new Vector2(Size.x * _numberOfPixelsPerUnityUnit, Size.y * _numberOfPixelsPerUnityUnit);
            }
        }

        public Texture2D Texture { get; private set; }

        public string Url { get; private set; }

        public Texture2D VideoTexture { get; private set; }

        public void Init(Texture2D viewportTexture, float width, float height) {

            Init(viewportTexture, width, height, null);
        }

        public void Init(Texture2D viewportTexture, float width, float height, Texture2D videoTexture) {

            Texture = viewportTexture;
            VideoTexture = videoTexture;
            Size = new Vector2(width, height);
            _log("Init() width: {0}, height: {1}", width.ToString("n4"), height.ToString("n4"));
        }

        public void Blur() {

            _log("Blur()");
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

        public void CanGoBack(Action<bool> callback) {

            _log("CanGoBack()");
            callback(false);
        }

        public void CanGoForward(Action<bool> callback) {

            _log("CanGoForward()");
            callback(false);
        }

    #if NET_4_6
        public Task<byte[]> CaptureScreenshot() {

            var task = new TaskCompletionSource<byte[]>();
            CaptureScreenshot(task.SetResult);
            return task.Task;
        }
    #endif // NET_4_6

        public void CaptureScreenshot(Action<byte[]> callback) {

            _log("CaptureScreenshot()");
            callback(new byte[0]);
        }

        public void Click(Vector2 point) {

            _log("Click({0})", point.ToString("n4"));
        }

        public void DisableViewUpdates() {

            _log("DisableViewUpdates()");
        }

        public void Dispose() {

            _log("Dispose()");
        }

        public void EnableViewUpdates() {

            _log("EnableViewUpdates()");
        }

    #if NET_4_6
        public Task<string> ExecuteJavaScript(string javaScript) {

            var task = new TaskCompletionSource<string>();
            ExecuteJavaScript(javaScript, task.SetResult);
            return task.Task;
        }
    #else
        public void ExecuteJavaScript(string javaScript) {

            _log("ExecuteJavaScript(\"{0}...\")", javaScript.Substring(0, 25));
        }
    #endif // NET_4_6

        public void ExecuteJavaScript(string javaScript, Action<string> callback) {

            _log("ExecuteJavaScript(\"{0}...\")", javaScript.Substring(0, 25));
            callback("");
        }

        public void Focus() {

            _log("Focus()");
        }

        #if NET_4_6
            public Task<byte[]> GetRawTextureData() {

                var task = new TaskCompletionSource<byte[]>();
                GetRawTextureData(task.SetResult);
                return task.Task;
            }
        #endif // NET_4_6

        public void GetRawTextureData(Action<byte[]> callback) {

            _log("GetRawTextureData()");
            callback(new byte[0]);
        }

        public void GoBack() {

            _log("GoBack()");
        }

        public void GoForward() {

            _log("GoForward()");
        }

        public void HandleKeyboardInput(string input) {

            _log("HandleKeyboardInput(\"{0}\")", input);
        }

        public virtual void LoadHtml(string html) {

            Url = html.Substring(0, 25);
            _log("LoadHtml(\"{0}...\")", Url);
            if (UrlChanged != null) {
                UrlChanged(this, new UrlChangedEventArgs(Url, "Title", UrlActionType.Load));
            }
            if (LoadProgressChanged != null) {
                LoadProgressChanged(this, new ProgressChangedEventArgs(ProgressChangeType.Finished, 1.0f));
            }
        }

        public virtual void LoadUrl(string url) {

            LoadUrl(url, null);
        }

        public virtual void LoadUrl(string url, Dictionary<string, string> additionalHttpHeaders) {

            Url = url;
            _log("LoadUrl(\"{0}\")", url);
            if (UrlChanged != null) {
                UrlChanged(this, new UrlChangedEventArgs(url, "Title", UrlActionType.Load));
            }
            if (LoadProgressChanged != null) {
                LoadProgressChanged(this, new ProgressChangedEventArgs(ProgressChangeType.Finished, 1.0f));
            }
        }

        public void PostMessage(string data) {

            _log("PostMessage(\"{0}\")", data);
        }

        public void Reload() {

            _log("Reload()");
        }

        public void Resize(float width, float height) {

            Size = new Vector2(width, height);
            _log("Resize({0}, {1})", width.ToString("n4"), height.ToString("n4"));
        }

        public void Scroll(Vector2 delta) {

            _log("Scroll({0})", delta.ToString("n4"));
        }

        public void Scroll(Vector2 delta, Vector2 mousePosition) {

            _log("Scroll({0}, {1})", delta.ToString("n4"), mousePosition.ToString("n4"));
        }

        public void SetResolution(int pixelsPerUnityUnit) {

            _numberOfPixelsPerUnityUnit = pixelsPerUnityUnit;
            _log("SetResolution({0})", pixelsPerUnityUnit);
        }

        public void ZoomIn() {

            _log("ZoomIn()");
        }

        public void ZoomOut() {

            _log("ZoomOut()");
        }

        int _numberOfPixelsPerUnityUnit = Config.NumberOfPixelsPerUnityUnit;

        void _log(string message, params object[] args) {

            Debug.LogFormat("[MockWebView] " + message, args);
        }
    }
}
