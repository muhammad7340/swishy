#version 330
// Kelp leaves: the albedo alpha cuts out the leaf shapes, and a simple
// two-sided diffuse term (|N.L|) lights both faces of the duplicated mesh.

uniform sampler2D albedoMap;
uniform vec3 lightDir;      // towards the light, object space
uniform float ambient;      // share of the colour that is always lit

in block
{
  vec2 uv;
  vec3 n;
} inPs;

out vec4 fragColor;

void main()
{
  vec4 albedo = texture(albedoMap, inPs.uv);
  if (albedo.a < 0.5)
    discard;
  float diffuse = abs(dot(normalize(inPs.n), normalize(lightDir)));
  fragColor = vec4(albedo.rgb * (ambient + (1.0 - ambient) * diffuse), 1.0);
}
