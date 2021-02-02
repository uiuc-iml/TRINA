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
using UnityEngine.UI;

namespace Vuplex.WebView {

    /// <summary>
    /// Extends `ViewportMaterialView` by emitting pointer events.
    /// </summary>
    public class WebViewPrefabView : ViewportMaterialView,
                                     IBeginDragHandler,
                                     IDragHandler,
                                     IPointerDownHandler,
                                     IPointerUpHandler,
                                     IScrollHandler {

        public event EventHandler<EventArgs<PointerEventData>> BeganDrag;

        public event EventHandler<EventArgs<PointerEventData>> Dragged;

        public event EventHandler<EventArgs<PointerEventData>> PointerDown;

        public event EventHandler<EventArgs<PointerEventData>> PointerUp;

        public event EventHandler<EventArgs<PointerEventData>> Scrolled;

        /// <see cref="IBeginDragHandler"/>
        public void OnBeginDrag(PointerEventData data) {

            if (BeganDrag != null) {
                BeganDrag(this, new EventArgs<PointerEventData>(data));
            }
        }

        /// <see cref="IDragHandler"/>
        public void OnDrag(PointerEventData eventData) {

            if (Dragged != null) {
                Dragged(this, new EventArgs<PointerEventData>(eventData));
            }
        }

        /// <see cref="IPointerDownHandler"/>
        public virtual void OnPointerDown(PointerEventData eventData) {

            if (PointerDown != null) {
                PointerDown(this, new EventArgs<PointerEventData>(eventData));
            }
        }

        /// <see cref="IPointerUpHandler"/>
        public virtual void OnPointerUp(PointerEventData eventData) {

            if (PointerUp != null) {
                PointerUp(this, new EventArgs<PointerEventData>(eventData));
            }
        }

        /// <see cref="IScrollHandler"/>
        public void OnScroll(PointerEventData eventData) {

            if (Scrolled != null) {
                Scrolled(this, new EventArgs<PointerEventData>(eventData));
            }
        }
    }
}

