#version 300 es
precision highp float;

in vec3 vNormal;
in vec3 vPosition;

out vec4 fragColor;

uniform vec3 uLightPos;
uniform vec3 uCameraPos;

void main() {
    // Material properties
    vec3 baseColor = vec3(0.02, 0.02, 0.02); // Very dark gray, almost black
    float roughness = 0.1; // Low roughness for a glossy finish
    float metallic = 0.9; // High metallic value for a reflective look

    // Normalize vectors
    vec3 N = normalize(vNormal);
    vec3 L = normalize(uLightPos - vPosition);
    vec3 V = normalize(uCameraPos - vPosition);
    vec3 H = normalize(L + V);

    // Diffuse term (Lambert)
    float NdotL = max(dot(N, L), 0.0);
    vec3 diffuse = baseColor * NdotL;

    // Specular term (Blinn-Phong)
    float NdotH = max(dot(N, H), 0.0);
    float specularStrength = pow(NdotH, (1.0 - roughness) * 256.0);
    vec3 specular = vec3(specularStrength);

    // Fresnel term (Schlick's approximation)
    float F0 = mix(0.04, 1.0, metallic);
    float fresnel = F0 + (1.0 - F0) * pow(1.0 - max(dot(H, V), 0.0), 5.0);

    // Combine terms
    vec3 finalColor = diffuse + specular * fresnel;

    // Output final color
    fragColor = vec4(finalColor, 1.0);
}