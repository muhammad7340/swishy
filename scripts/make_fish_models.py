#!/usr/bin/env python3
"""Write the ambient-fish models (models/fish_*): real fish, swimming.

The meshes are CC0 photogrammetry scans by ffish.asia / floraZia.com
(Sketchfab "ffishAsia-and-floraZia"), mirrored as GLB in
github.com/lisa-1002/fishing3d (vendor/fish/real). With --from-scans DIR
(holding the scan_*.glb files) each scan is converted in Blender by
scripts/fish_scan_to_parts.py: reference cube dropped, turned to face +x
(+z up), centred, scaled to its length and decimated, one OBJ and one
512 px texture. Only single-piece scans convert cleanly: scans made of several
textured patches fell apart when decimated (hollow, scrambled fish), so they
are not used. Without --from-scans, only the SDF is rewritten.

Each model's visuals use the shared SwimFish shaders (models/fish_common):
the fish swims a closed path around the model origin, so a world include
only places the path's centre (layout_decor.py does that per tank).
visibility_flags 4 hides them from the gpu_lidar and the eye cameras, like
the water, so the robot's perception and maps are unchanged.

Usage:
    python3 scripts/make_fish_models.py [--from-scans DIR]
"""
import glob
import os
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
MODELS = os.path.join(ROOT, "src", "fishy_fish_navigation", "models")

# name: (scan, length m, decimation target per scan, common name, Sketchfab
#        source, swim parameters)
SPECIES = {
    "fish_carp": ("scan_carp", 0.32, 6000, "Carp (Cyprinus carpio)",
                  "https://sketchfab.com/3d-models/cc0-carp-cyprinus-carpio-6b404d20bab34fa99fba848060c42ca7",
                  dict(waveAmp=0.10, headAmp=0.15, waveFreq=1.2, waveCount=0.7, pattern=0,
                       radii=(1.0, 0.7), speed=0.13, bob=0.04)),
    "fish_fusilier": ("scan_fusilier", 0.20, 5000, "Fusilier",
                      "https://sketchfab.com/ffishAsia-and-floraZia",
                      dict(waveAmp=0.12, headAmp=0.15, waveFreq=2.4, waveCount=0.8, pattern=1,
                           radii=(0.55, 0.40), speed=0.30, bob=0.03)),
    "fish_puffer": ("scan_puffer_vermic", 0.16, 5000, "Vermiculated puffer",
                    "https://sketchfab.com/ffishAsia-and-floraZia",
                    dict(waveAmp=0.05, headAmp=0.30, waveFreq=2.5, waveCount=0.5, pattern=1,
                         radii=(0.40, 0.30), speed=0.16, bob=0.03)),
    "fish_bleak": ("scan_horsemouth", 0.15, 4000, "Pale bleak (Zacco platypus)",
                   "https://sketchfab.com/3d-models/cc0-pale-bleak-zacco-platypus-b1ddc3a76994493b91204b287bba569c",
                   dict(waveAmp=0.12, headAmp=0.15, waveFreq=2.8, waveCount=0.8, pattern=0,
                        radii=(0.45, 0.30), speed=0.35, bob=0.03)),
    "fish_pompano": ("scan_pompano", 0.20, 5000, "Pompano",
                     "https://sketchfab.com/ffishAsia-and-floraZia",
                     dict(waveAmp=0.08, headAmp=0.20, waveFreq=1.6, waveCount=0.6, pattern=0,
                          radii=(0.60, 0.45), speed=0.20, bob=0.04)),
}

CREDIT = ("Photogrammetry scan by ffish.asia / floraZia.com (Sketchfab "
          "ffishAsia-and-floraZia), CC0 1.0, via github.com/lisa-1002/fishing3d")


def convert(scans, name, scan, length, target):
    out = os.path.join(MODELS, name)
    subprocess.run(["blender", "-b", "--factory-startup", "--python",
                    os.path.join(ROOT, "scripts", "fish_scan_to_parts.py"), "--",
                    os.path.join(scans, scan + ".glb"), out, str(length), str(target)], check=True)
    from PIL import Image
    # 128 px: the fish shader's texture has no mipmaps, and a finer photo
    # atlas shimmers into salt-and-pepper noise when the fish is small on
    # screen; pre-shrinking does the mipmap's job for normal viewing distances
    for p in glob.glob(os.path.join(out, "materials", "textures", "*.jpg")):
        Image.open(p).convert("RGB").resize((128, 128), Image.LANCZOS).save(p, "JPEG", quality=90)
    for p in glob.glob(os.path.join(out, "meshes", "*.obj")):   # 0.1 mm is plenty
        lines = []
        for line in open(p):
            t = line.split()
            if t and t[0] == "v":
                line = "v " + " ".join(f"{float(x):.4f}" for x in t[1:4]) + "\n"
            elif t and t[0] in ("vt", "vn"):
                line = t[0] + " " + " ".join(f"{float(x):.3f}" for x in t[1:]) + "\n"
            elif t and t[0] in ("o", "s", "#", "mtllib", "usemtl"):
                continue
            lines.append(line)
        open(p, "w").writelines(lines)


def write_model(name, scan, length, title, source, p):
    d = os.path.join(MODELS, name)
    link = os.path.join(d, "materials", "programs")
    if not os.path.lexists(link):
        os.symlink("../../fish_common/materials/programs", link)
    parts = sorted(os.path.basename(f)[:-4] for f in glob.glob(os.path.join(d, "meshes", "*.obj")))
    visuals = []
    for part in parts:
        visuals.append(f"""
      <visual name="{part}">
        <geometry><mesh><uri>meshes/{part}.obj</uri></mesh></geometry>
        <cast_shadows>false</cast_shadows>
        <visibility_flags>4</visibility_flags>
        <plugin filename="gz-sim-shader-param-system" name="gz::sim::systems::ShaderParam">
          <shader language="glsl">
            <vertex>materials/programs/SwimFish_vs_330.glsl</vertex>
            <fragment>materials/programs/SwimFish_fs_330.glsl</fragment>
          </shader>
          <param><shader>vertex</shader><name>worldviewproj_matrix</name></param>
          <param><shader>vertex</shader><name>world_matrix</name></param>
          <param><shader>vertex</shader><name>t</name><value>TIME</value></param>
          <param><shader>vertex</shader><name>bodyLength</name><type>float</type><value>{length}</value></param>
          <param><shader>vertex</shader><name>waveAmp</name><type>float</type><value>{p['waveAmp']}</value></param>
          <param><shader>vertex</shader><name>headAmp</name><type>float</type><value>{p['headAmp']}</value></param>
          <param><shader>vertex</shader><name>waveFreq</name><type>float</type><value>{p['waveFreq']}</value></param>
          <param><shader>vertex</shader><name>waveCount</name><type>float</type><value>{p['waveCount']}</value></param>
          <param><shader>vertex</shader><name>pattern</name><type>float</type><value>{p['pattern']}</value></param>
          <param><shader>vertex</shader><name>radii</name><type>float_array</type><value>{p['radii'][0]} {p['radii'][1]}</value></param>
          <param><shader>vertex</shader><name>speed</name><type>float</type><value>{p['speed']}</value></param>
          <param><shader>vertex</shader><name>bob</name><type>float</type><value>{p['bob']}</value></param>
          <param><shader>fragment</shader><name>albedoMap</name><type>texture</type><value>materials/textures/{part}.jpg</value></param>
          <param><shader>fragment</shader><name>lightDir</name><type>float_array</type><value>0.3 -0.4 0.87</value></param>
          <param><shader>fragment</shader><name>ambient</name><type>float</type><value>0.5</value></param>
        </plugin>
      </visual>""")
    pattern = "figure-eight" if p["pattern"] else "ellipse"
    open(os.path.join(d, "model.sdf"), "w").write(f"""<?xml version="1.0"?>
<!-- GENERATED by scripts/make_fish_models.py. {title}, {length} m long.
     {CREDIT}.
     Swims a {pattern} of half-axes {p['radii'][0]} x {p['radii'][1]} m around the model
     origin (SwimFish shaders in fish_common); visual only, hidden from the
     gpu_lidar and eye cameras (visibility_flags 4). -->
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">{"".join(visuals)}
    </link>
  </model>
</sdf>
""")
    open(os.path.join(d, "model.config"), "w").write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author><name>ffish.asia / floraZia.com (scan), CC0 1.0</name></author>
  <description>
    {title}: {CREDIT}. Source: {source}.
    Converted by scripts/make_fish_models.py (decimated, 128 px texture).
  </description>
</model>
""")
    print(f"{name}: {len(parts)} part(s)")


if __name__ == "__main__":
    scans = sys.argv[sys.argv.index("--from-scans") + 1] if "--from-scans" in sys.argv else None
    for name, (scan, length, target, title, source, params) in SPECIES.items():
        if scans:
            convert(scans, name, scan, length, target)
        write_model(name, scan, length, title, source, params)
