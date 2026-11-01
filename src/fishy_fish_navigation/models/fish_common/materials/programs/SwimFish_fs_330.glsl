#version 330
// Ambient fish: the scan's photographed colour, lit from above with a
// two-sided diffuse term (thin fins are seen from both sides).

uniform sampler2D albedoMap;
uniform vec3 lightDir;      // towards the light, model frame
uniform float ambient;      // share of the colour that is always lit

in block
{
  vec2 uv;
  vec3 n;
} inPs;

out vec4 fragColor;

void main()
{
  vec3 albedo = texture(albedoMap, inPs.uv).rgb;
  float diffuse = abs(dot(normalize(inPs.n), normalize(lightDir)));
  fragColor = vec4(albedo * (ambient + (1.0 - ambient) * diffuse), 1.0);
}
