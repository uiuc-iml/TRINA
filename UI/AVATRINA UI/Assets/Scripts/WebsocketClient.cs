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
        public Candidate​ candidate;
        public int room;
        public Offer offer;
        public Answer answer;


        [System.Serializable]
        public class Answer
        {
            public string type = "answer";
            public string sdp;

            public static WsMessage ToWsMessage(RTCSessionDescription desc)
            {
                return new WsMessage
                {
                    type = "answer",
                    answer = new Answer
                    {
                        sdp = desc.sdp
                    },
                };
            }
        }

        [System.Serializable]
        public class Offer
        {
            public string type = "offer";
            public string sdp;

            public static RTCSessionDescription ToRTCSessionDescription(Offer offer)
            {
                return new RTCSessionDescription
                {
                    type = RTCSdpType.Offer,
                    sdp = offer.sdp
                };
            }
        }
        [System.Serializable]
        public class Candidate​
        {
            public string candidate;
            public string sdpMid;
            public int sdpMLineIndex;
            public static RTCIceCandidate​ ToRTCIceCandidate​(Candidate​ candidate)
            {
                return new RTCIceCandidate​
                {
                    candidate = candidate.candidate,
                    sdpMid = candidate.sdpMid,
                    sdpMLineIndex = candidate.sdpMLineIndex
                };
            }
            public static WsMessage ToWsMessage(RTCIceCandidate candidate)
            {
                return new WsMessage
                {
                    type = "candidate",
                    candidate = new Candidate​
                    {
                        candidate = candidate.candidate,
                        sdpMid = candidate.sdpMid,
                        sdpMLineIndex = candidate.sdpMLineIndex
                    },
                };
            }

        }
    }

  


    public class WsMessageCandidate
    {
        public string type ="candidate";
        public string candidate;
    }

    private WebSocket ws;
    public bool offerReceived = false;
    public RTCSessionDescription sessionDesc = new RTCSessionDescription();
    public List<RTCIceCandidate​> iceCandidates = new List<RTCIceCandidate​> { };


    void Start()
    {

        Debug.Log("initiate websocket");
        //initiate websocket Connection
        ws = new WebSocket("ws://130.126.138.139:9000");
 

        ws.OnMessage += (sender, e) =>
        {
            Debug.Log(e.Data);
            var data = JsonUtility.FromJson<WsMessage>(e.Data);
            Debug.Log($"onMessage{data.type}");
            switch (data.type)
            {
                case "ready":
                    break;
                case "offer":
                    sessionDesc = WsMessage.Offer.ToRTCSessionDescription(data.offer);
                    offerReceived = true;
                    break;
                case "answer":
                    break;
                case "candidate":
                    iceCandidates.Add(WsMessage.Candidate.ToRTCIceCandidate(data.candidate));
                    break;
                default:
                    break;
            }
        };

        ws.OnOpen += (sender, e) =>
        {
            Debug.Log("onOpen");
            WsMessage message = new WsMessage
            {
                type = "ready",
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
