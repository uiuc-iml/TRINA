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

namespace Vuplex.WebView {

    public class BridgeMessageHandler {

        public event EventHandler<EventArgs<StringWithIdBridgeMessage>> JavaScriptResultReceived;

        public event EventHandler<EventArgs<string>> MessageEmitted;

        public event EventHandler<EventArgs<string>> TitleChanged;

        public event EventHandler<UrlChangedEventArgs> UrlChanged;

        public event EventHandler<EventArgs<Rect>> VideoRectChanged;

        public Rect VideoRect {
            get {
                return _videoRect;
            }
        }

        public void HandleMessage(string serializedMessage) {

            var messageType = BridgeMessage.ParseType(serializedMessage);
            switch (messageType) {
                case "vuplex.webview.javaScriptResult":
                    var message = JsonUtility.FromJson<StringWithIdBridgeMessage>(serializedMessage);
                    if (JavaScriptResultReceived != null) {
                        JavaScriptResultReceived(this, new EventArgs<StringWithIdBridgeMessage>(message));
                    }
                    break;
                case "vuplex.webview.titleChanged":
                    var title = StringBridgeMessage.ParseValue(serializedMessage);
                    if (TitleChanged != null) {
                        TitleChanged(this, new EventArgs<string>(title));
                    }
                    break;
                case "vuplex.webview.urlChanged":
                    var action = JsonUtility.FromJson<UrlChangedMessage>(serializedMessage).urlAction;
                    if (UrlChanged != null) {
                        UrlChanged(this, new UrlChangedEventArgs(action.Url, action.Title, action.Type));
                    }
                    break;
                case "vuplex.webview.videoRectChanged":
                    _handleVideoRectChangedMessage(serializedMessage);
                    break;

                default:
                    if (MessageEmitted != null) {
                        MessageEmitted(this, new EventArgs<string>(serializedMessage));
                    }
                    break;
            }
        }

        Rect _videoRect = new Rect(0, 0, 0, 0);

        void _handleVideoRectChangedMessage(string serializedMessage) {

            var value = JsonUtility.FromJson<VideoRectChangedMessage>(serializedMessage).value;
            var newRect = value.rect.toRect();
            if (_videoRect != newRect) {
                _videoRect = newRect;
                if (VideoRectChanged != null) {
                    VideoRectChanged(this, new EventArgs<Rect>(newRect));
                }
            }
        }
    }
}

