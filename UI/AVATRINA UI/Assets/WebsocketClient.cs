using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;
using Unity.WebRTC;
public class WebsocketClient : MonoBehaviour
{
    // Start is called before the first frame update

    [System.Serializable]
    public class WsMessage
    {
        public string type;
        public string candidate;
        public int room;
        public Offer offer;
        public string answer;
        [System.Serializable]
        public class Offer
        {
            public string type = "offer";
            public string sdp;
        }
    }

  

    [System.Serializable]
    public class WsMessageAnswer
    {
        public string type = "answer";
        public Answer answer;
        [System.Serializable]
        public class Answer
        {
            public string type = "answer";
            public string sdp;
        }
        public static WsMessageAnswer ToWsMessageAnswer(RTCSessionDescription desc)
        {
            return new WsMessageAnswer
            {
                answer = new Answer
                {
                    sdp = desc.sdp
                },
            };
        }
    }

    public class WsMessageRoom
    {
        public string type = "ready";
        public int room;
    }

    public class WsMessageCandidate
    {
        public string type ="candidate";
        public string candidate;
    }

    private WebSocket ws;
    public bool offerReceived = false;
    public RTCSessionDescription sessionDesc = new RTCSessionDescription();



    void Start()
    {

        Debug.Log("initiate websocket");
        //initiate websocket and webrtc peerConnection
        ws = new WebSocket("ws://130.126.138.139:9000");
 

        ws.OnMessage += (sender, e) =>
        {
            Debug.Log("onMessage");
            var data = JsonUtility.FromJson<WsMessage>(e.Data);
            Debug.Log(data.type);
            switch (data.type)
            {
                case "ready":
                    break;
                case "offer":
                    sessionDesc = new RTCSessionDescription
                    {
                        type = RTCSdpType.Offer,
                        sdp = data.offer.sdp
                    };
                    offerReceived = true;
                    break;
                case "answer":
                    break;
                case "candidate":
                    break;
                default:
                    break;
            }
        };

        ws.OnOpen += (sender, e) =>
        {
            Debug.Log("onOpen");
            WsMessageRoom message = new WsMessageRoom
            {
                room = 1
            };
            ws.Send(JsonUtility.ToJson(message));
        };


        ws.OnClose += (sender, e) =>
        {
            Debug.Log("onClose");
            Debug.Log("WebSocket closed with reason: " + e.Reason);
        };


        ws.Connect();

    }

    public void SendWS(string data)
    {
        ws.Send(data);
    }


}
