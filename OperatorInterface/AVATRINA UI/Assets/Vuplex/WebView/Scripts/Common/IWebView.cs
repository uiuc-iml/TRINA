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
using UnityEngine;

#if NET_4_6
using System.Threading.Tasks;
#endif // NET_4_6

namespace Vuplex.WebView {

    /// <summary>
    /// `IWebView` is the primary interface for loading and interacting with web content.
    /// </summary>
    /// <remarks>
    /// `WebViewPrefab` takes care of creating one for you and hooking it up to the materials
    /// in its prefab. If you want to create an `IWebView` outside of the prefab (to connect
    /// to your own custom GameObject) you can use `Web.CreateWebView()`.
    /// </remarks>
    public interface IWebView {

        /// <summary>
        /// Indicates that the page load percentage changed.
        /// </summary>
        event EventHandler<ProgressChangedEventArgs> LoadProgressChanged;

        /// <summary>
        /// Indicates that JavaScript running in the page used the `window.vuplex.postMessage`
        /// JavaScript API to emit a message to the Unity application.
        /// </summary>
        /// <example>
        /// JavaScript Example for sending a message:
        /// ```
        /// function sendMessageToCSharp() {
        ///   // This object passed to `postMessage()` is automatically serialized as JSON
        ///   // and is emitted via the C# MessageEmitted event. This API mimics the window.postMessage API.
        ///   window.vuplex.postMessage({ type: 'greeting', message: 'Hello from JavaScript!' });
        /// }
        ///
        /// if (window.vuplex) {
        ///   // The window.vuplex object has already been initialized after page load,
        ///   // so we can go ahead and send the message.
        ///   sendMessageToCSharp();
        /// } else {
        ///   // The window.vuplex object hasn't been initialized yet because the page is still
        ///   // loading, so add an event listener to send the message once it's initialized.
        ///   window.addEventListener('vuplexready', sendMessageToCSharp);
        /// }
        /// ```
        /// </example>
        event EventHandler<EventArgs<string>> MessageEmitted;

        /// <summary>
        /// Indicates that the page failed to load. This can happen, for instance,
        /// if DNS is unable to resolve the hostname.
        /// </summary>
        event EventHandler PageLoadFailed;

        /// <summary>
        /// Indicates that the page's title changed.
        /// </summary>
        event EventHandler<EventArgs<string>> TitleChanged;

        /// <summary>
        /// Indicates that the URL of the webview changed, either
        /// due to user interaction or JavaScript.
        /// </summary>
        event EventHandler<UrlChangedEventArgs> UrlChanged;

        /// <summary>
        /// Indicates that the rect of the playing video changed.
        /// </summary>
        /// <remarks>
        /// Note that `WebViewPrefab` automatically handles this event for you.
        /// </remarks>
        event EventHandler<EventArgs<Rect>> VideoRectChanged;

        /// <summary>
        /// The webview's resolution in pixels per Unity unit.
        /// </summary>
        /// <seealso cref="SizeInPixels"/>
        int Resolution { get; }

        /// <summary>
        /// The webview's current size in Unity units.
        /// </summary>
        Vector2 Size { get; }

        /// <summary>
        /// The webview's current size in pixels.
        /// </summary>
        /// <seealso cref="Resolution"/>
        Vector2 SizeInPixels { get; }

        /// <summary>
        /// The texture for the webview's web content.
        /// </summary>
        Texture2D Texture { get; }

        /// <summary>
        /// The current URL.
        /// </summary>
        string Url { get; }

        /// <summary>
        /// The texture for the webview's video content.
        /// </summary>
        Texture2D VideoTexture { get; }

        /// <summary>
        /// Initializes a newly created webview with the given viewport and video textures created with
        /// `Web.CreateTexture()` and the dimensions in Unity units.
        /// </summary>
        /// <remarks>
        /// Note that `WebViewPrefab` automatically
        /// calls this method for you, so you only need to invoke this directly if you manually create a WebView
        /// outside of the prefab using `Web.CreateWebView()`.
        /// </remarks>
        void Init(Texture2D viewportTexture, float width, float height, Texture2D videoTexture);

        /// <summary>
        /// Like the other `Init()` method, but with video support disabled.
        /// </summary>
        void Init(Texture2D viewportTexture, float width, float height);

        /// <summary>
        /// Makes the webview relinquish focus.
        /// </summary>
        void Blur();

    #if NET_4_6
        /// <summary>
        /// Checks whether the webview can go back with a call to `GoBack()`.
        /// </summary>
        Task<bool> CanGoBack();

        /// <summary>
        /// Checks whether the webview can go forward with a call to `GoForward()`.
        /// </summary>
        Task<bool> CanGoForward();

    #endif // NET_4_6

        /// <summary>
        /// Like the other version of `CanGoBack()`, except it uses a callback
        /// instead of a `Task` in order to be compatible with versions of .NET before 4.0.
        /// </summary>
        void CanGoBack(Action<bool> callback);

        /// <summary>
        /// Like the other version of `CanGoForward()`, except it uses a callback
        /// instead of a `Task` in order to be compatible with versions of .NET before 4.0.
        /// </summary>
        void CanGoForward(Action<bool> callback);

    #if NET_4_6
        /// <summary>
        /// Returns a PNG image of the content visible in the webview.
        /// Note that screenshots do not include video content, which appears black.
        /// </summary>
        Task<byte[]> CaptureScreenshot();

    #endif // NET_4_6

        /// <summary>
        /// Like the other version of `CaptureScreenshot()`, except it uses a callback
        /// instead of a `Task` in order to be compatible with versions of .NET before 4.0.
        /// </summary>
        void CaptureScreenshot(Action<byte[]> callback);

        /// <summary>
        /// Clicks at the given point in the webpage.
        /// </summary>
        /// <param name="point">
        /// The x and y components of the point are values
        /// between 0 and 1 that are normalized to the width and height, respectively. For example,
        /// `point.x = x in Unity units / width in Unity units`.
        /// Like in the browser, the origin is in the upper-left corner,
        /// the positive direction of the y-axis is down, and the positive
        /// direction of the x-axis is right.
        /// </param>
        void Click(Vector2 point);

        /// <summary>
        /// Disables the webview from rendering to its texture.
        /// </summary>
        void DisableViewUpdates();

        /// <summary>
        /// Destroys the webview, releasing all of its resources.
        /// </summary>
        void Dispose();

        /// <summary>
        /// Re-enables rendering after a call to `DisableViewUpdates()`.
        /// </summary>
        void EnableViewUpdates();

    #if NET_4_6
        /// <summary>
        /// Executes the given script in the context of the webpage's main frame
        /// and returns the result.
        /// </summary>
        /// <remarks>
        /// When targeting .NET 4.6 or newer, this method returns a `Task<string>`.
        /// When targeting an older version of .NET, this method returns void.
        /// </remarks>
        Task<string> ExecuteJavaScript(string javaScript);
    #else
        /// <summary>
        /// Executes the given script in the context of the webpage's main frame.
        /// </summary>
        /// <remarks>
        /// When targeting .NET 4.6 or newer, this method returns a `Task<string>`.
        /// When targeting an older version of .NET, this method returns void.
        /// </remarks>
        void ExecuteJavaScript(string javaScript);
    #endif // NET_4_6

        /// <summary>
        /// Executes the given script in the context of the webpage's main frame
        /// and calls the given callback with the result.
        /// </summary>
        /// <remarks>
        /// This method is functionally equivalent to the version of `ExecuteJavaScript()`
        /// that returns a `Task`, except it uses a callback instead of a `Task` in order
        /// to be compatible with versions of .NET before 4.0.
        /// </remarks>
        void ExecuteJavaScript(string javaScript, Action<string> callback);

        /// <summary>
        /// Makes the webview take focus.
        /// </summary>
        void Focus();

    #if NET_4_6
        /// <summary>
        /// A replacement for [`Texture2D.GetRawTextureData()`](https://docs.unity3d.com/ScriptReference/Texture2D.GetRawTextureData.html)
        /// for IWebView.Texture. Note that the texture data excludes video content, which appears black.
        /// </summary>
        /// <remarks>
        /// Unity's `Texture2D.GetRawTextureData()` currently does not work for textures created with
        /// `Texture2D.CreateExternalTexture()`. So, this method serves as a replacement by providing
        /// the equivalent functionality. You can load the bytes returned by this method into another
        /// texture using [`Texture2D.LoadRawTextureData()`](https://docs.unity3d.com/ScriptReference/Texture2D.LoadRawTextureData.html).
        /// </remarks>
        Task<byte[]> GetRawTextureData();

    #endif // NET_4_6

        /// <summary>
        /// Like the other version of `GetRawTextureData()`, except it uses a callback
        /// instead of a `Task` in order to be compatible with versions of .NET before 4.0.
        /// </summary>
        void GetRawTextureData(Action<byte[]> callback);

        /// <summary>
        /// Navigates back to the previous page in the webview's history.
        /// </summary>
        void GoBack();

        /// <summary>
        /// Navigates forward to the next page in the webview's history.
        /// </summary>
        void GoForward();

        /// <summary>
        /// Dispatches a keystroke to the webview.
        /// </summary>
        /// <param name="key">
        /// A key can either be a single character representing
        /// a unicode character (e.g. "A", "b", "?") or a [JavaScript Key value](https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent/key/Key_Values)
        /// (e.g. "ArrowUp", "Enter").
        /// </param>
        void HandleKeyboardInput(string key);

        /// <summary>
        /// Loads the webpage contained in the given HTML string.
        /// </summary>
        /// <![CDATA[
        /// Example:
        /// ```
        /// webView.LoadHtml(@"<!DOCTYPE html>
        ///     <html>
        ///         <head>
        ///             <title>Test Page</title>
        ///             <style>
        ///                 h1 {
        ///                     font-family: Helvetica, Arial, Sans-Serif;
        ///                 }
        ///             </style>
        ///         </head>
        ///         <body>
        ///             <h1>LoadHtml Example</h1>
        ///             <script>
        ///                 console.log('This page was loaded!');
        ///             </script>
        ///         </body>
        ///     </html>"
        /// );
        /// ```
        /// ]]>
        void LoadHtml(string html);

        /// <summary>
        /// Loads the given URL.
        /// </summary>
        /// <param name="url">
        /// An http or https URL to a webpage or a file URL that points to a local file
        /// in the Android or iOS application bundle. For example, you can assemble
        /// a file URL on Android if you know the path to the resource:
        ///
        /// `file:///android_asset/my-static-files/my-webpage.html`
        ///
        /// On iOS, you can use `iOSWebView.GetFileUrlForBundleResource()` to
        /// ask the system for a file URL to a given bundle resource.
        /// You can use  has a static  method that can do
        /// </param>
        void LoadUrl(string url);

        /// <summary>
        /// Like `LoadUrl(string url)`, but also sends the given HTTP request headers
        /// when loading the URL.
        /// </summary>
        void LoadUrl(string url, Dictionary<string, string> additionalHttpHeaders);

        /// <summary>
        /// Reloads the current page.
        /// </summary>
        void Reload();

        /// <summary>
        /// Resizes the webview to the dimensions given in Unity units.
        /// </summary>
        /// <remarks>
        /// Note that if you're using `WebViewPrefab`, you should call
        /// `WebViewPrefab.Resize()` instead.
        /// </remarks>
        void Resize(float width, float height);

        /// <summary>
        /// Posts a message that JavaScript within the webview can listen for
        /// using `window.vuplex.addEventListener('message', function(message) {})`.
        /// </summary>
        /// <param name="data">
        /// String that is passed as the data property of the message object.
        /// </param>
        void PostMessage(string data);

        /// <summary>
        /// Scrolls the webview's top-level body by the given delta.
        /// If you want to scroll a specific section of the page,
        /// see `Scroll(Vector2 scrollDelta, Vector2 mousePosition)` instead.
        /// </summary>
        /// <param name="scrollDelta">
        /// The scroll delta in Unity units. Because the browser's origin
        /// is in the upper-left corner, the y-axis' positive direction
        /// is down and the x-axis' positive direction is right.
        /// </param>
        void Scroll(Vector2 scrollDelta);

        /// <summary>
        /// Scrolls by the given delta at the given mouse position.
        /// </summary>
        /// <param name="scrollDelta">
        /// The scroll delta in Unity units. Because the browser's origin
        /// is in the upper-left corner, the y-axis' positive direction
        /// is down and the x-axis' positive direction is right.
        /// </param>
        /// <param name="mousePosition">
        /// The mouse position at the time of the scroll. The x and y components of are values
        /// between 0 and 1 that are normalized to the width and height, respectively. For example,
        /// `point.x = x in Unity units / width in Unity units`.
        /// </param>
        void Scroll(Vector2 scrollDelta, Vector2 mousePosition);

        /// <summary>
        /// Sets the webview's resolution in pixels per Unity unit.
        /// </summary>
        /// <remarks>
        /// The default resolution is 1300 pixels per Unity unit.
        /// This method is useful, for example, if you want web content to
        /// appear larger or smaller. Setting a lower resolution decreases
        /// the pixel density, but has the effect of making web content appear larger.
        /// Setting a higher resolution increases the pixel density, but has the effect
        /// of making content appear smaller.
        /// </remarks>
        void SetResolution(int pixelsPerUnityUnit);

        /// <summary>
        /// Zooms into the currently loaded web content.
        /// </summary>
        /// <remarks>
        /// Note that the zoom level gets reset when a new page is loaded.
        /// </remarks>
        void ZoomIn();

        /// <summary>
        /// Zooms back out after a previous call to `ZoomIn()`.
        /// </summary>
        /// <remarks>
        /// Note that the zoom level gets reset when a new page is loaded.
        /// </remarks>
        void ZoomOut();
    }
}
