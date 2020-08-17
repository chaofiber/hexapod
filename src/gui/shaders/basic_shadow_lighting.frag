#version 330 core

out vec4 FragColor;
uniform vec3 objectColor;
uniform sampler2DShadow shadowMap;                                                               
uniform float bias;
#include "compute_shading.frag"

float CalcShadowFactor(vec4 LightSpacePos)
{
	LightSpacePos.xyz = 0.5*LightSpacePos.xyz+0.5;
	float visibility=textureProj(shadowMap, vec4(LightSpacePos.xy,  LightSpacePos.z-bias,LightSpacePos.w));

	return visibility;
}

vec3 computeShadowMapShading(){
	vec3 ambient = computeAmbientComponent();
	vec3 diffuse = computeDiffuseComponent();
	vec3 specular = computeSpecularComponent();

	float shadowFactor = CalcShadowFactor(lightSpacePos);
	return ambient + shadowFactor * (diffuse + specular);
}

void main()
{
	vec3 result =  computeShadowMapShading() * objectColor;
    FragColor = vec4(result, 1.0);
} 
