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
// For Android, it renders from OES_external_image textures, which require special
// OpenGLES extensions and a special texture sampler.
Shader "Vuplex/Android Viewport Shader" {
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
                #pragma only_renderers gles gles3
                #extension GL_OES_EGL_image_external : require
                #extension GL_OES_EGL_image_external_essl3 : enable

                #pragma multi_compile ___ _STEREOMODE_TOPBOTTOM _STEREOMODE_LEFTRIGHT
                #pragma multi_compile ___ FLIP_X
                #pragma multi_compile ___ FLIP_Y

                precision mediump int;
                precision mediump float;

                #ifdef VERTEX
                    uniform mat4 video_matrix;
                    uniform int unity_StereoEyeIndex;
                    uniform float _OverrideStereoToMono;
                    varying vec2 uv;

                    void main() {
                        gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
                        vec4 untransformedUV = gl_MultiTexCoord0;
                        #ifdef FLIP_X
                            untransformedUV.x = 1.0 - untransformedUV.x;
                        #endif  // FLIP_X
                        #ifdef FLIP_Y
                            untransformedUV.y = 1.0 - untransformedUV.y;
                        #endif  // FLIP_Y
                        #ifdef _STEREOMODE_TOPBOTTOM
                            untransformedUV.y *= 0.5;
                            if (unity_StereoEyeIndex == 1 && _OverrideStereoToMono != 1.0) {
                                untransformedUV.y += 0.5;
                            }
                        #endif  // _STEREOMODE_TOPBOTTOM
                        #ifdef _STEREOMODE_LEFTRIGHT
                            untransformedUV.x *= 0.5;
                            if (unity_StereoEyeIndex != 0 && _OverrideStereoToMono != 1.0) {
                                untransformedUV.x += 0.5;
                            }
                        #endif  // _STEREOMODE_LEFTRIGHT

                        uv = (video_matrix * untransformedUV).xy;
                    }
                #endif  // VERTEX

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
