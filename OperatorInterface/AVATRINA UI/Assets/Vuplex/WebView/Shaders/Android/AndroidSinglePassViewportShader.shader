// <copyright file="VideoUnlitShader.cs" company="Google Inc.">
// Copyright (C) 2017 Google Inc. All Rights Reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//    limitations under the License.
// </copyright>

// This shader is a modified version of VideoUnlitShader.cs from the Unity Google VR SDK.
// It's the same as AndroidViewportShader.shader, except its vertex shader is
// modified to work with single-pass stereo rendering on Android.
Shader "Vuplex/Android Single Pass Stereo Viewport Shader" {
    Properties {
        _MainTex ("Base (RGB)", 2D) = "white" {}
        [KeywordEnum(None, TopBottom, LeftRight)] _StereoMode ("Stereo mode", Float) = 0
        [Toggle(FLIP_X)] _FlipX ("Flip X", Float) = 0
        [Toggle(FLIP_Y)] _FlipY ("Flip Y", Float) = 0
        _Gamma ("Gamma for Gamma Correction", Range(0.01, 3.0)) = 0.6

        [Header(Properties set programmatically)]
        _VideoCutoutRect("Video Cutout Rect", Vector) = (0, 0, 0, 0)
        _CropRect("Crop Rect", Vector) = (0, 0, 0, 0)
        _OverrideStereoToMono ("Override Stereo to Mono", Float) = 0
        _EnableGammaCorrection ("Enable Gamma Correction", Float) = 0
    }
    SubShader {
        Pass {
            Tags { "Queue" = "Transparent" "RenderType" = "Transparent" }

            Lighting Off
            ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha

            GLSLPROGRAM
                #pragma multi_compile ___ _STEREOMODE_TOPBOTTOM _STEREOMODE_LEFTRIGHT
                #pragma multi_compile ___ FLIP_X
                #pragma multi_compile ___ FLIP_Y

                #ifdef VERTEX
                    #version 300 es
                    #extension GL_OVR_multiview2 : require
                    #extension GL_OES_EGL_image_external : require
                    #extension GL_OES_EGL_image_external_essl3 : enable

                    uniform vec4 hlslcc_mtx4x4unity_ObjectToWorld[4];
                    uniform vec4 _MainTex_ST;
                    uniform float _OverrideStereoToMono;
                    layout(std140) uniform UnityStereoGlobals {
                        vec4 hlslcc_mtx4x4unity_StereoMatrixP[8];
                        vec4 hlslcc_mtx4x4unity_StereoMatrixV[8];
                        vec4 hlslcc_mtx4x4unity_StereoMatrixInvV[8];
                        vec4 hlslcc_mtx4x4unity_StereoMatrixVP[8];
                        vec4 hlslcc_mtx4x4unity_StereoCameraProjection[8];
                        vec4 hlslcc_mtx4x4unity_StereoCameraInvProjection[8];
                        vec4 hlslcc_mtx4x4unity_StereoWorldToCamera[8];
                        vec4 hlslcc_mtx4x4unity_StereoCameraToWorld[8];
                        vec3 unity_StereoWorldSpaceCameraPos[2];
                        vec4 unity_StereoScaleOffset[2];
                    };
                    layout(num_views = 2) in;
                    in highp vec4 in_POSITION0;
                    in highp vec2 in_TEXCOORD0;
                    out highp vec2 uv;
                    vec4 u_xlat0;
                    int u_xlati1;
                    vec4 u_xlat2;

                    void main() {
                        vec2 untransformedUV = in_TEXCOORD0;
                        u_xlati1 = int(gl_ViewID_OVR) << 2;

                        #ifdef FLIP_X
                            untransformedUV.x = 1.0 - untransformedUV.x;
                        #endif  // FLIP_X
                        #ifdef FLIP_Y
                            untransformedUV.y = 1.0 - untransformedUV.y;
                        #endif  // FLIP_Y
                        #ifdef _STEREOMODE_TOPBOTTOM
                            untransformedUV.y *= 0.5;
                            if (u_xlati1 != 0 && _OverrideStereoToMono != 1.0) {
                                untransformedUV.y += 0.5;
                            }
                        #endif // _STEREOMODE_TOPBOTTOM
                        #ifdef _STEREOMODE_LEFTRIGHT
                        untransformedUV.x *= 0.5;
                            if (u_xlati1 != 0 && _OverrideStereoToMono != 1.0) {
                                untransformedUV.x += 0.5;
                            }
                        #endif  // _STEREOMODE_LEFTRIGHT

                        // Handle single pass stereo rendering
                        uv.xy = untransformedUV.xy * _MainTex_ST.xy + _MainTex_ST.zw;
                        u_xlat0 = in_POSITION0.yyyy * hlslcc_mtx4x4unity_ObjectToWorld[1];
                        u_xlat0 = hlslcc_mtx4x4unity_ObjectToWorld[0] * in_POSITION0.xxxx + u_xlat0;
                        u_xlat0 = hlslcc_mtx4x4unity_ObjectToWorld[2] * in_POSITION0.zzzz + u_xlat0;
                        u_xlat0 = u_xlat0 + hlslcc_mtx4x4unity_ObjectToWorld[3];
                        u_xlat2 = u_xlat0.yyyy * hlslcc_mtx4x4unity_StereoMatrixVP[(u_xlati1 + 1)];
                        u_xlat2 = hlslcc_mtx4x4unity_StereoMatrixVP[u_xlati1] * u_xlat0.xxxx + u_xlat2;
                        u_xlat2 = hlslcc_mtx4x4unity_StereoMatrixVP[(u_xlati1 + 2)] * u_xlat0.zzzz + u_xlat2;
                        gl_Position = hlslcc_mtx4x4unity_StereoMatrixVP[(u_xlati1 + 3)] * u_xlat0.wwww + u_xlat2;
                    }
                #endif // VERTEX

                #ifdef FRAGMENT

                    vec3 gammaCorrect(vec3 v, float gamma) {
                        return pow(v, vec3(1.0/gamma));
                    }

                    vec4 gammaCorrect(vec4 v, float gamma) {
                        return vec4(gammaCorrect(v.xyz, gamma), v.w);
                    }

                    uniform float _Gamma;
                    uniform samplerExternalOES _MainTex;
                    uniform vec4 _VideoCutoutRect;
                    uniform vec4 _CropRect;
                    uniform float _EnableGammaCorrection;
                    varying vec2 uv;

                    void main() {

                        gl_FragColor = texture2D(_MainTex, uv);
                        if (_EnableGammaCorrection == 1.0) {
                            gl_FragColor = gammaCorrect(gl_FragColor, _Gamma);
                        }

                        // Make the pixels transparent if they fall within the video rect cutout and the they're black.
                        // Keeping non-black pixels allows the video controls to still show up on top of the video.
                        float cutoutWidth = _VideoCutoutRect.z;
                        float cutoutHeight = _VideoCutoutRect.w;

                        #ifdef FLIP_Y
                            float nonflippedY = 1.0 - uv.y;
                        #else
                            float nonflippedY = uv.y;
                        #endif  // FLIP_Y
                        bool pointIsInCutout = cutoutWidth != 0.0 && cutoutHeight != 0.0 && uv.x >= _VideoCutoutRect.x && uv.x <= _VideoCutoutRect.x + cutoutWidth && nonflippedY >= _VideoCutoutRect.y && nonflippedY <= _VideoCutoutRect.y + cutoutHeight;
                        if (pointIsInCutout && gl_FragColor == vec4(0.0, 0.0, 0.0, 1.0)) {
                            gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
                        }

                        float cropWidth = _CropRect.z;
                        float cropHeight = _CropRect.w;
                        bool pointIsOutsideOfCrop = cropWidth != 0.0 && cropHeight != 0.0 && (uv.x < _CropRect.x || uv.x > _CropRect.x + cropWidth || uv.y < _CropRect.y || uv.y > _CropRect.y + cropHeight);
                        if (pointIsOutsideOfCrop) {
                            gl_FragColor = vec4(0.0, 0.0, 0.0, 0.0);
                        }
                    }
                #endif  // FRAGMENT
            ENDGLSL
        }
    }
    Fallback "Unlit/Texture"
}
