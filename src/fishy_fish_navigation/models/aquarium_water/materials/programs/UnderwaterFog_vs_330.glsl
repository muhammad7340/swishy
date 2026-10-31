#version 330
// Camera-centred fog shells (meshes/fog_shells.obj). Each inward-facing shell
// is moved onto the camera, so an object at distance d sits behind ~d/spacing
// shells and is tinted like exponential fog: 1 - exp(-density * d).

in vec4 vertex;
uniform mat4 worldviewproj_matrix;
uniform vec3 camera_position_object_space;

out gl_PerVertex
{
  vec4 gl_Position;
};

out block
{
  vec3 pos;
  float camZ;
} outVs;

void main()
{
  vec4 P = vec4(vertex.xyz + camera_position_object_space, 1.0);
  gl_Position = worldviewproj_matrix * P;
  outVs.pos = P.xyz;
  outVs.camZ = camera_position_object_space.z;
}
