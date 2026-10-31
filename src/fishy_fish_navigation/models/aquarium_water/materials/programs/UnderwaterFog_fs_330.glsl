#version 330
// Fog only while the camera is under water, and only inside the tank.

uniform vec4 fogColor;
uniform float density;    // 1/m
uniform float spacing;    // shell spacing in m, must match the mesh
uniform vec2 halfSize;    // tank interior half extents in m
uniform float waterZ;     // surface height in the model frame

in block
{
  vec3 pos;
  float camZ;
} inPs;

out vec4 fragColor;

void main()
{
  if (inPs.camZ > waterZ || inPs.pos.z > waterZ ||
      abs(inPs.pos.x) > halfSize.x || abs(inPs.pos.y) > halfSize.y)
    discard;
  fragColor = vec4(fogColor.rgb, 1.0 - exp(-density * spacing));
}
