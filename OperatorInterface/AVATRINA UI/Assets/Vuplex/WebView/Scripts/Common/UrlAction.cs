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

namespace Vuplex.WebView {

    [Serializable]
    public class UrlAction {

        public UrlAction() {}

        public UrlAction(string url, string title, string type) {
            Url = url;
            Title = title;
            Type = type;
        }

        public string Url;

        public string Title;

        /// <see cref="UrlActionType"/>
        public string Type;
    }
}
