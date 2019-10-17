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
using UnityEngine.EventSystems;

namespace Vuplex.WebView {
    /// <summary>
    /// A prefab that makes it easy to create and interact with an `IWebView`.
    /// </summary>
    /// <remarks>
    /// `WebViewPrefab` takes care of wiring the `IWebView` up to its prefab's textures and input handlers, so all
    /// you need to do is load some web content from a URL or HTML string,
    /// and then the user can view and interact with the web content.
    ///
    /// `WebViewPrefab` automatically handles pointer click, scroll, and drag events (also used for scrolling) from
    /// Unity's input event system.
    ///
    ///  If your use case requires a high degree of customization, you can instead create an `IWebView`
    /// outside of the prefab using `Web.CreateWebView()` and initialize it with a texture created
    /// with `Web.CreateTexture()`.
    /// </remarks>
    /// <example>
    /// Example:
    ///
    /// ```
    /// // Create a 0.5 x 0.5 instance of of the prefab
    /// var webViewPrefab = WebViewPrefab.Instantiate(0.5f, 0.5f);
    /// // Position the prefab how we want it
    /// webViewPrefab.transform.parent = transform;
    /// webViewPrefab.transform.localPosition = new Vector3(0, 0f, 0.5f);
    /// webViewPrefab.transform.LookAt(transform);
    /// // Load a URL once the prefab finishes initializing
    /// webViewPrefab.Initialized += (sender, e) => {
    ///     webViewPrefab.WebView.LoadUrl("https://vuplex.com");
    /// };
    /// ```
    /// </example>
    public class WebViewPrefab : MonoBehaviour {

        /// <summary>
        /// Indicates that the prefab was clicked. Note that the prefab automatically
        /// calls the `IWebView.Click()` method for you.
        /// </summary>
        public event EventHandler Clicked;

        /// <summary>
        /// Indicates that the prefab finished initializing,
        /// so its `WebView` property is ready to use.
        /// </summary>
        public event EventHandler Initialized;

        /// <summary>
        /// Clicking is enabled by default, but can be disabled by
        /// setting this property to `false`.
        /// </summary>
        public bool ClickingEnabled {
            get { return _clickingEnabled; }
            set { _clickingEnabled = value; }
        }

        /// <summary>
        /// In order to prevent dragging-to-scroll from triggering unwanted
        /// clicks, a click is discarded if the pointer drags beyond this
        /// threshold before PointerUp occurs. The default threshold is 0.15.
        /// </summary>
        public float DragToScrollThreshold {
            get { return _dragToScrollThreshold; }
            set { _dragToScrollThreshold = value; }
        }

        /// <summary>
        /// Scrolling is enabled by default, but can be disabled by
        /// setting this property to `false`.
        /// </summary>
        public bool ScrollingEnabled {
            get { return _scrollingEnabled; }
            set { _scrollingEnabled = value; }
        }

        /// <summary>
        /// Allows the scroll sensitivity to be adjusted.
        /// The default sensitivity is 0.001.
        /// </summary>
        public static float ScrollSensitivity {
            get { return _scrollSensitivity; }
            set { _scrollSensitivity = value; }
        }

        /// <summary>
        /// A reference to the prefab's `IWebView` instance, which
        /// is available after the `Initialized` event is raised.
        /// Before initialization is complete, this property is `null`.
        /// </summary>
        public IWebView WebView { get { return _webView; }}

        /// <summary>
        /// Creates a new instance of the `WebViewPrefab` prefab with the given
        /// dimensions in Unity units and initializes it asynchronously.
        /// </summary>
        /// <remarks>
        /// The `WebView` property is available after initialization completes,
        /// which is indicated by the `Initialized` event.
        /// </remarks>
        public static WebViewPrefab Instantiate(float width, float height) {

            return Instantiate(width, height, new WebViewOptions());
        }

        /// <summary>
        /// Like `Instantiate(float, float)`, except it also accepts an object
        /// of options flags that can be used to alter the generated webview's behavior.
        /// </summary>
        public static WebViewPrefab Instantiate(float width, float height, WebViewOptions options) {

            var prefabPrototype = (GameObject) Resources.Load("WebViewPrefab");
            var viewGameObject = (GameObject) Instantiate(prefabPrototype);
            var webViewPrefab = viewGameObject.GetComponent<WebViewPrefab>();
            webViewPrefab.Init(width, height, options);
            return webViewPrefab;
        }

        /// <summary>
        /// Asynchronously initializes the WebViewPrefab using the width and height
        /// set via the Unity editor.
        /// </summary>
        /// <remarks>
        /// You only need to call this method
        /// directly if you place a WebViewPrefab prefab in your scene using the Unity editor;
        /// it's not needed if you instantiate the prefab programmatically using
        /// `Instantiate()`. This method asynchronously initializes
        /// the `WebView property`, which is available after the Initialized
        /// event is fired.
        /// </remarks>
        public void Init() {

            // The top-level WebViewPrefab object's scale must be (1, 1),
            // so the scale that was set via the editor is transferred from WebViewPrefab
            // to WebViewPrefabResizer, and WebViewPrefab is moved to compensate
            // for how WebViewPrefabResizer is moved in _setViewSize.
            var localScale = transform.localScale;
            var localPosition = transform.localPosition;
            transform.localScale = new Vector3(1, 1, localScale.z);
            var offsetMagnitude = 0.5f * localScale.x;
            transform.localPosition = transform.localPosition + Quaternion.Euler(transform.localEulerAngles) * new Vector3(offsetMagnitude, 0, 0);
            Init(localScale.x, localScale.y);
        }

        /// <summary>
        /// Asynchronously initializes the WebViewPrefab to the specified size.
        /// </summary>
        /// <remarks>
        /// You only need to call this method
        /// directly if you place a WebViewPrefab prefab in your scene using the Unity editor;
        /// it's not needed if you instantiate the prefab programmatically using
        /// `Instantiate()`. This method asynchronously initializes
        /// the `WebView property`, which is available after the Initialized
        /// event is fired.
        /// </remarks>
        public virtual void Init(float width, float height) {

            Init(width, height, new WebViewOptions());
        }

        /// <summary>
        /// Like `Init(float, float)`, except it also accepts an object
        /// of options flags that can be used to alter the webview's behavior.
        /// </summary>
        public virtual void Init(float width, float height, WebViewOptions options) {

            #if UNITY_ANDROID && !UNITY_EDITOR
                AndroidWebView.AssertWebViewIsAvailable();
            #endif
            _options = options;
            _viewResizer = transform.GetChild(0);
            _view = _getView();
            _setViewSize(width, height);
            _view.BeganDrag += View_BeganDrag;
            _view.Dragged += View_Dragged;
            _view.PointerDown += View_PointerDown;
            _view.PointerUp += View_PointerUp;
            _view.Scrolled += View_Scrolled;
            Web.CreateMaterial(viewportMaterial => {
                _view.SetMaterial(viewportMaterial);
                _initWebViewIfReady();
            });
            _videoRectPositioner = _viewResizer.Find("VideoRectPositioner");
            _videoLayer = _videoRectPositioner.GetComponentInChildren<ViewportMaterialView>();
            if (options.disableVideo) {
                _videoLayerDisabled = true;
                _videoRectPositioner.gameObject.SetActive(false);
                _initWebViewIfReady();
            } else {
                Web.CreateVideoMaterial(videoMaterial => {
                    if (videoMaterial == null) {
                        _videoLayerDisabled = true;
                        _videoRectPositioner.gameObject.SetActive(false);
                    } else {
                        _videoLayer.SetMaterial(videoMaterial);
                    }
                    _initWebViewIfReady();
                });
            }
        }

        /// <summary>
        /// Destroys the `WebViewPrefab` and its children. Note that you don't have
        /// to call this method if you destroy the `WebViewPrefab`'s parent with
        /// `Object.Destroy()`.
        /// </summary>
        public void Destroy() {

            UnityEngine.Object.Destroy(gameObject);
        }

        /// <summary>
        /// Resizes the prefab mesh and webview to the given
        /// dimensions in Unity units.
        /// </summary>
        public void Resize(float width, float height) {

            if (_webView != null) {
                _webView.Resize(width, height);
            }
            _setViewSize(width, height);
        }

        Vector2 _beginningDragPoint;
        Vector2 _beginningDragRatioPoint;
        bool _clickingEnabled = true;
        bool _clickIsPending;
        float _dragToScrollThreshold = 0.15f;
        WebViewOptions _options;
        Vector2 _previousDragPoint;
        bool _scrollingEnabled = true;
        static float _scrollSensitivity = 0.005f;
        ViewportMaterialView _videoLayer;
        bool _videoLayerDisabled;
        Transform _videoRectPositioner;
        protected WebViewPrefabView _view;
        Transform _viewResizer;
        protected IWebView _webView;

        void View_BeganDrag(object sender, EventArgs<PointerEventData> e) {

            _beginningDragRatioPoint = _convertToScreenPosition(e.Value.pointerCurrentRaycast);
            _beginningDragPoint = _convertRatioPointToUnityUnits(_beginningDragRatioPoint);
            _previousDragPoint = _beginningDragPoint;
        }

        void View_Dragged(object sender, EventArgs<PointerEventData> e) {

            if (e.Value.pointerCurrentRaycast.worldPosition == new Vector3(0, 0, 0)) {
                // This happens when the user drags off of the screen.
                return;
            }
            var point = _convertToScreenPosition(e.Value.pointerCurrentRaycast);
            var newDragPoint = _convertRatioPointToUnityUnits(point);
            var dragDelta = _previousDragPoint - newDragPoint;
            _scrollIfNeeded(dragDelta, _beginningDragRatioPoint);
            _previousDragPoint = newDragPoint;

            // Check whether to cancel a pending viewport click so that drag-to-scroll
            // doesn't unintentionally trigger a click.
            if (_clickIsPending) {
                var totalDragDelta = _beginningDragPoint - newDragPoint;
                var dragReachedScrollThreshold = Math.Abs(totalDragDelta.x) > _dragToScrollThreshold ||  Math.Abs(totalDragDelta.y) > _dragToScrollThreshold;
                if (dragReachedScrollThreshold) {
                    _clickIsPending = false;
                }
            }
        }

        protected virtual void View_PointerDown(object sender, EventArgs<PointerEventData> e) {

            if (_clickingEnabled) {
                _clickIsPending = true;
            }
        }

        protected virtual void View_PointerUp(object sender, EventArgs<PointerEventData> e) {

            if (!(_clickingEnabled && _clickIsPending)) {
                return;
            }
            _clickIsPending = false;
            var point = _convertToScreenPosition(e.Value.pointerPressRaycast);
            _webView.Click(point);
            if (Clicked != null) {
                Clicked(this, EventArgs.Empty);
            }
        }

        void View_Scrolled(object sender, EventArgs<PointerEventData> e) {

            var scaledScrollDelta = new Vector2(
                -e.Value.scrollDelta.x * _scrollSensitivity,
                -e.Value.scrollDelta.y * _scrollSensitivity
            );
            var mousePosition = _convertToScreenPosition(e.Value.pointerCurrentRaycast);
            _scrollIfNeeded(scaledScrollDelta, mousePosition);
        }

        Vector2 _convertRatioPointToUnityUnits(Vector2 point) {

            var unityUnitsX = _viewResizer.transform.localScale.x * point.x;
            var unityUnitsY = _viewResizer.transform.localScale.y * point.y;
            return new Vector2(unityUnitsX, unityUnitsY);
        }

        /**
        * @returns point
        * @returns point.x - ratio between 0 and 1
        * @returns point.y - ratio between 0 and 1
        */
        protected Vector2 _convertToScreenPosition(RaycastResult raycastResult) {

            var localPosition = _viewResizer.transform.InverseTransformPoint(raycastResult.worldPosition);
            return new Vector2(1 - localPosition.x, -1 * localPosition.y);
        }

        void _scrollIfNeeded(Vector2 scrollDelta, Vector2 mousePosition) {

            if (!_scrollingEnabled) {
                return;
            }
            if (scrollDelta.x == 0 && scrollDelta.y == 0) {
                // This can happen after the user drags the cursor off of the screen.
                return;
            }
            _webView.Scroll(scrollDelta, mousePosition);
        }

        protected virtual WebViewPrefabView _getView() {

            return transform.GetComponentInChildren<WebViewPrefabView>();
        }

        void _initWebViewIfReady() {

            if (_view.Texture == null || (!_videoLayerDisabled && _videoLayer.Texture == null)) {
                // Wait until both views' textures are ready.
                return;
            }
            _webView = Web.CreateWebView();
            if (!_options.disableVideo) {
                _webView.VideoRectChanged += (sender, e) => _setVideoRect(e.Value);
            }
            _webView.Init(_view.Texture, _viewResizer.localScale.x, _viewResizer.localScale.y, _videoLayer.Texture);
            _setVideoRect(new Rect(0, 0, 0, 0));
            if (Initialized != null) {
                Initialized(this, EventArgs.Empty);
            }
        }

        void OnDestroy() {

            if (_webView != null) {
                _webView.Dispose();
            }
        }

        void _setVideoRect(Rect videoRect) {

            _view.SetCutoutRect(videoRect);
            // The origins of the prefab and the video rect are in their top-right
            // corners instead of their top-left corners.
            _videoRectPositioner.localPosition = new Vector3(
                1 - (videoRect.x + videoRect.width),
                -1 * videoRect.y,
                _videoRectPositioner.localPosition.z
            );
            _videoRectPositioner.localScale = new Vector3(videoRect.width, videoRect.height, _videoRectPositioner.localScale.z);

            // This code applies a cropping rect to the video layer's shader based on what part of the video (if any)
            // falls outside of the viewport and therefore needs to be hidden. Note that the dimensions here are divided
            // by the videoRect's width or height, because in the videoLayer shader, the width of the videoRect is 1
            // and the height is 1 (i.e. the dimensions are normalized).
            float videoRectXMin = Math.Max(0, - 1 * videoRect.x / videoRect.width);
            float videoRectYMin = Math.Max(0, -1 * videoRect.y / videoRect.height);
            float videoRectXMax = Math.Min(1, (1 - videoRect.x) / videoRect.width);
            float videoRectYMax = Math.Min(1, (1 - videoRect.y) / videoRect.height);
            var videoCropRect = Rect.MinMaxRect(videoRectXMin, videoRectYMin, videoRectXMax, videoRectYMax);
            if (videoCropRect == new Rect(0, 0, 1, 1)) {
                // The entire video rect fits within the viewport, so set the cropt rect to zero to disable it.
                videoCropRect = new Rect(0, 0, 0, 0);
            }
            _videoLayer.SetCropRect(videoCropRect);
        }

        void _setViewSize(float width, float height) {

            var depth = _viewResizer.localScale.z;
            _viewResizer.localScale = new Vector3(width, height, depth);
            var localPosition = _viewResizer.localPosition;
            // Set the view resizer so that its middle aligns with the middle of this parent class's gameobject.
            localPosition.x = width * -0.5f;
            _viewResizer.localPosition = localPosition;
        }
    }
}

