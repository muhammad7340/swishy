#version 330
// Light shafts (meshes/god_rays.obj). u = ray index + across-strip coordinate,
// v = 0 at the surface .. 1 at the sand.

in vec4 vertex;
in vec3 normal;
in vec4 uv0;
uniform mat4 worldviewproj_matrix;
uniform vec3 camera_position_object_space;
uniform float t;

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec2 uv;
  vec3 eyeVec;
  vec3 normal;
  float camZ;
} outVs;

void main()
{
  vec4 P = vertex;
  // slow sway of the lower end, one phase per ray
  float seed = floor(uv0.x);
  P.x += 0.04 * uv0.y * sin(0.5 * t + 1.7 * seed);
  P.y += 0.03 * uv0.y * sin(0.37 * t + 2.3 * seed);
  gl_Position = worldviewproj_matrix * P;
  outVs.uv = uv0.xy;
  outVs.eyeVec = P.xyz - camera_position_object_space;
  outVs.normal = normal;
  outVs.camZ = camera_position_object_space.z;
}
