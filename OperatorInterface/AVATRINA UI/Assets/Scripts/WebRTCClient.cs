//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;
//using Unity.WebRTC;
//using System.Linq;
//using static WebsocketClient;

//public class WebRTCClient : MonoBehaviour
//{

//    private RTCAnswerOptions AnswerOptions = new RTCAnswerOptions
//    {
//        iceRestart = false,
//    };

//    private int candiateCount = 0;

//    RTCConfiguration GetSelectedSdpSemantics()
//    {
//        RTCConfiguration config = default;
//        config.iceServers = new RTCIceServer[]
//        {
//            new RTCIceServer { urls = new string[] { "stun:stun.l.google.com:19302" } }
//        };

//        return config;
//    }

//    private RTCPeerConnection pc;

//    private void Awake()
//    {
//        // Initialize WebRTC
//        WebRTC.Initialize();
//    }


   

//    // Start is called before the first frame update
//    void Start()
//    {
//        var configuration = GetSelectedSdpSemantics();
//        pc = new RTCPeerConnection(ref configuration);
//        pc.OnIceCandidate = e =>
//        {
//            Debug.Log(e.candidate);
//            if (!string.IsNullOrEmpty(e.candidate))
//            {
//                WsMessage message = WsMessage.Candidate.ToWsMessage(e);
//                gameObject.GetComponent<WebsocketClient>().SendWS(JsonUtility.ToJson(message));
//            }

//        };

//        pc.OnDataChannel = e =>
//        {
//            Debug.Log(e.Label);
//        };

//        pc.OnTrack = e =>
//        {
//            Debug.Log("on track");
//            Debug.Log(e.Track);
           
//        };

        

//    }

//    void Update()
//    {
//        if (gameObject.GetComponent<WebsocketClient>().offerReceived)
//        {
//            var sessionDesc = gameObject.GetComponent<WebsocketClient>().sessionDesc;
//            StartCoroutine(Signal(sessionDesc));
//            gameObject.GetComponent<WebsocketClient>().offerReceived = false;
//        }

//        if (gameObject.GetComponent<WebsocketClient>().iceCandidates.Count > candiateCount)
//        {
//            var candidate = gameObject.GetComponent<WebsocketClient>().iceCandidates.Last();
//            StartCoroutine(AddCandidate(candidate));
//            candiateCount++;
//        }
//    }



//    private void OnDestroy()
//    {
//        WebRTC.Finalize();
//    }

//    IEnumerator Signal(RTCSessionDescription desc)
//    {
//        var op1 = pc.SetRemoteDescription(ref desc);
//        yield return op1;
//        var op2 = pc.CreateAnswer(ref AnswerOptions);
//        yield return op2;
//        var op3 = pc.SetLocalDescription(ref op2.desc);
//        yield return op3;
//        WsMessage message = WsMessage.Answer.ToWsMessage(op2.desc);
//        gameObject.GetComponent<WebsocketClient>().SendWS(JsonUtility.ToJson(message));
//    }

//    IEnumerator AddCandidate(RTCIceCandidate​ candidate)
//    {
//        pc.AddIceCandidate(ref candidate);
//        yield return null;
//    }


//}
