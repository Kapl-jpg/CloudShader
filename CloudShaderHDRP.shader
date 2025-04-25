Shader "Custom/CloudShaderHDRP"
{
    Properties
    {
        _Color ("Color", Color) = (1,1,1,1)
        _Density ("Density", Range(0, 1)) = 0.5
        _EdgeFade ("Edge Fade", Range(0, 1)) = 0.3
        _Softness ("Softness", Range(0, 1)) = 0.5
        _NoiseScale ("Noise Scale", Float) = 10.0
        _EdgeNoiseScale ("Edge Noise Scale", Float) = 5.0
        _NoiseStrength ("Noise Strength", Range(0, 1)) = 0.5
        _EdgeNoiseStrength ("Edge Noise Strength", Range(0, 1)) = 0.3
        _NoiseOctaves ("Noise Octaves", Range(1, 8)) = 3
        _NoiseSpeed ("Noise Speed", Vector) = (0.1, 0.1, 0, 0)
        _MainLightDir ("Main Light Direction", Vector) = (0,1,0,0)
        _TessellationFactor ("Tessellation Factor", Range(1, 64)) = 4
        _TessNoiseScale ("Tess Noise Scale", Float) = 5.0
        _TessNoiseStrength ("Tess Noise Strength", Range(0, 1)) = 0.5
        _AlphaMultiplier ("Alpha Multiplier", Range(0, 2)) = 1.5
        _Brightness ("Brightness", Range(1, 3)) = 1.5
    }
    SubShader
    {
        Tags { "RenderPipeline"="HDRenderPipeline" "RenderType"="Transparent" "Queue"="Transparent+100" }
        LOD 100

        Pass
        {
            Name "Forward"
            Tags { "LightMode" = "Forward" }

            ZWrite Off
            Cull Off
            Blend SrcAlpha OneMinusSrcAlpha
            ZTest LEqual

            HLSLPROGRAM
            #pragma target 5.0
            #pragma vertex vert
            #pragma hull hull
            #pragma domain domain
            #pragma fragment frag
            #pragma multi_compile _ DOTS_INSTANCING_ON
            #pragma multi_compile _ _MAIN_LIGHT_SHADOWS _MAIN_LIGHT_SHADOWS_CASCADE
            #pragma multi_compile _ _SHADOWS_OFF

            #include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Common.hlsl"
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/ShaderLibrary/ShaderVariables.hlsl"
            #include "Packages/com.unity.render-pipelines.high-definition/Runtime/RenderPipeline/ShaderPass/ShaderPass.cs.hlsl"

            // Параметры
            float4 _Color;
            float _Density;
            float _EdgeFade;
            float _Softness;
            float _NoiseScale;
            float _EdgeNoiseScale;
            float _NoiseStrength;
            float _EdgeNoiseStrength;
            float _NoiseOctaves;
            float4 _NoiseSpeed;
            float4 _MainLightDir;
            float _TessellationFactor;
            float _TessNoiseScale;
            float _TessNoiseStrength;
            float _AlphaMultiplier;
            float _Brightness;

            struct appdata
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float3 positionOS : TEXCOORD2;
                UNITY_VERTEX_INPUT_INSTANCE_ID
                UNITY_VERTEX_OUTPUT_STEREO
            };

            struct tessControlPoint
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct tessFactors
            {
                float edge[3] : SV_TessFactor;
                float inside : SV_InsideTessFactor;
            };

            // Функция шума Перлина (2D версия) с октавами
            float2 random2(float2 p)
            {
                return frac(sin(float2(dot(p, float2(127.1, 311.7)), dot(p, float2(269.5, 183.3)))) * 43758.5453);
            }

            float perlinNoise(float2 p)
            {
                float2 i = floor(p);
                float2 f = frac(p);
                float2 u = f * f * (3.0 - 2.0 * f);

                float2 a = random2(i + float2(0.0, 0.0));
                float2 b = random2(i + float2(1.0, 0.0));
                float2 c = random2(i + float2(0.0, 1.0));
                float2 d = random2(i + float2(1.0, 1.0));

                float noise = lerp(lerp(dot(a, f - float2(0.0, 0.0)), dot(b, f - float2(1.0, 0.0)), u.x),
                                  lerp(dot(c, f - float2(0.0, 1.0)), dot(d, f - float2(1.0, 1.0)), u.x), u.y);
                return noise * 0.5 + 0.5; // Нормализуем в [0,1]
            }

            float fbm(float2 p, float scale)
            {
                float value = 0.0;
                float amplitude = 0.5;
                float frequency = 1.0;

                for (int i = 0; i < _NoiseOctaves; i++)
                {
                    value += amplitude * perlinNoise(p * frequency * scale);
                    frequency *= 2.0;
                    amplitude *= 0.5;
                }
                return value;
            }

            // Вершинный шейдер
            tessControlPoint vert(appdata input)
            {
                tessControlPoint output;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_TRANSFER_INSTANCE_ID(input, output);

                output.positionOS = input.positionOS;
                output.normalOS = input.normalOS;
                return output;
            }

            // Hull шейдер
            [domain("tri")]
            [partitioning("integer")]
            [outputtopology("triangle_cw")]
            [patchconstantfunc("hullConst")]
            [outputcontrolpoints(3)]
            tessControlPoint hull(InputPatch<tessControlPoint, 3> patch, uint id : SV_OutputControlPointID)
            {
                return patch[id];
            }

            tessFactors hullConst(InputPatch<tessControlPoint, 3> patch)
            {
                tessFactors factors;
                factors.edge[0] = _TessellationFactor;
                factors.edge[1] = _TessellationFactor;
                factors.edge[2] = _TessellationFactor;
                factors.inside = _TessellationFactor;
                return factors;
            }

            // Domain шейдер
            [domain("tri")]
            v2f domain(tessFactors factors, OutputPatch<tessControlPoint, 3> patch, float3 bary : SV_DomainLocation)
            {
                v2f output;
                UNITY_SETUP_INSTANCE_ID(patch[0]);
                UNITY_TRANSFER_INSTANCE_ID(patch[0], output);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);

                // Интерполяция позиций и нормалей
                float4 positionOS = patch[0].positionOS * bary.x + patch[1].positionOS * bary.y + patch[2].positionOS * bary.z;
                float3 normalOS = patch[0].normalOS * bary.x + patch[1].normalOS * bary.y + patch[2].normalOS * bary.z;

                // Применяем шум для деформации
                float2 noiseUV = positionOS.xz * _TessNoiseScale + _Time.y * _NoiseSpeed.xy;
                float noise = fbm(noiseUV, 1.0);
                positionOS.xyz += normalOS * noise * _TessNoiseStrength;

                // Преобразуем в мировое и экранное пространство
                float4 worldPos = float4(TransformObjectToWorld(positionOS.xyz), 1.0);
                output.positionWS = worldPos.xyz;
                output.normalWS = normalize(TransformObjectToWorldNormal(normalOS));
                output.positionCS = TransformWorldToHClip(output.positionWS);
                output.positionOS = positionOS.xyz;

                return output;
            }

            // Фрагментный шейдер
            float4 frag(v2f input) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

                // Освещение
                float3 normalWS = normalize(input.normalWS);
                float3 lightDir = normalize(_MainLightDir.xyz);
                float3 viewDir = normalize(GetWorldSpaceViewDir(input.positionWS));

                // Диффузное освещение
                float NdotL = max(0.4, dot(normalWS, lightDir)); // Увеличен минимальный порог
                float3 lighting = NdotL * _Color.rgb * _Brightness; // Добавлено усиление яркости

                // Прозрачность с растворяющимися краями
                float edge = abs(dot(normalWS, viewDir));
                float baseAlpha = _Density * (1.0 - _EdgeFade * pow(1.0 - edge, 2.0)); // Растворение на краях

                // Шум для пушистости
                float2 noiseUV = input.positionOS.xz * _NoiseScale + _Time.y * _NoiseSpeed.xy;
                float noise = fbm(noiseUV, 1.0);
                noise = noise * _NoiseStrength;

                // Шум для краев
                float2 edgeNoiseUV = input.positionOS.xz * _EdgeNoiseScale + _Time.y * _NoiseSpeed.xy;
                float edgeNoise = fbm(edgeNoiseUV, 1.0);
                edgeNoise = edgeNoise * _EdgeNoiseStrength * pow(1.0 - edge, 2.0); // Усиливаем шум на краях

                // Комбинируем альфа
                float alpha = baseAlpha + noise + edgeNoise;
                alpha = saturate(alpha * _AlphaMultiplier * (1.0 - _Softness));

                // Итоговой цвет
                float4 finalColor = float4(lighting, alpha);
                return finalColor;
            }
            ENDHLSL
        }
    }
    FallBack "Diffuse"
}