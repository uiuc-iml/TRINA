using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using WebSocketSharp;
using Unity.WebRTC;
public class WebsocketClient : MonoBehaviour
{
    // Start is called before the first frame update


    public class WsMessage
    {
        public string type;
        public string candidate;
        public int room;
        public string offer;
    }

    public class WsMessageOffer
    {
        public string type = "offer";
        public string offer;


    }

    public class WsMessageRoom
    {
        public string type = "room";
        public int room;
    }

    public class WsMessageCandidate
    {
        public string type ="candidate";
        public string candidate;
    }

    private WebSocket ws;
    private RTCPeerConnection pc;

    private RTCOfferOptions OfferOptions = new RTCOfferOptions
    {
        iceRestart = false,
        offerToReceiveAudio = true,
        offerToReceiveVideo = true
    };

    RTCConfiguration GetSelectedSdpSemantics()
    {
        RTCConfiguration config = default;
        config.iceServers = new RTCIceServer[]
        {
            new RTCIceServer { urls = new string[] { "stun:stun1.l.google.com:19302", "stun:stun2.1.google.com:19302" } }
        };

        return config;
    }




    private void Awake()
    {
        WebRTC.Initialize();
    }

    void Start()
    {
        Debug.Log("initiate websocket");
        //initiate websocket and webrtc peerConnection
        ws = new WebSocket("ws://130.126.138.139:9000");
        var configuration = GetSelectedSdpSemantics();
        pc = new RTCPeerConnection(ref configuration);
        
        pc.OnIceCandidate += e => {
            Debug.Log("111");
            Debug.Log(e.candidate);
            if (!string.IsNullOrEmpty(e.candidate)){
                WsMessageCandidate message = new WsMessageCandidate
                {
                    candidate = e.candidate
                };
                ws.Send(JsonUtility.ToJson(message));
            };
        };

        ws.OnMessage += (sender, e) => {
            Debug.Log("onMessage");
            var data = JsonUtility.FromJson<WsMessage>(e.Data);
            Debug.Log(data.type);
            switch (data.type) {
                case "ready":
                    //onReady(offerOnReady);
                    break;
                case "offer":
                    //onOffer(data.offer);
                    break;
                case "answer":
                    //onAnswer(data.answer);
                    break;
                case "candidate":
                    //onCandidate(data.candidate);
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

        pc.OnNegotiationNeeded += () =>
        {
           var offer = pc.CreateOffer(ref OfferOptions);
           var message = new WsMessageOffer { offer = "unity" };
           ws.Send(JsonUtility.ToJson(message));
           pc.SetLocalDescription(ref offer.desc);
        };

        ws.Connect();
     
    }

 
}
