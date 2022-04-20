import cv2
import os
import sys

script_template = '''material ArucoMarker{0}/Marker
{{
    technique
    {{
        pass
        {{
            texture_unit
            {{
                texture aruco_marker_{0}.png
            }}
        }}
    }}
}}
'''

model_template = """<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='aruco_marker_{0}'>

    <link name='marker'>
      <pose frame=''>0 0 0 0 0 0</pose>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.1 0.1 1e-5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://aruco_marker_{0}/materials/scripts</uri>
            <uri>model://aruco_marker_{0}/materials/textures</uri>
            <name>ArucoMarker{0}/Marker</name>
          </script>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 1 1 0</emissive>
          <shader type='vertex'>
            <normal_map>__default__</normal_map>
          </shader>
        </material>
        <pose>0 0 0 0 0 1.570796327</pose>
        <cast_shadows>1</cast_shadows>
        <transparency>0</transparency>
      </visual>
    </link>
    <static>1</static>
    <allow_auto_disable>1</allow_auto_disable>

  </model>
</sdf>"""

config_template = """<?xml version="1.0" ?>
<model>
    <name>aruco_marker_{0}</name>
    <version>1.0</version>
    <sdf version="1.6">model.sdf</sdf>
    <author>
        <name></name>
        <email></email>
    </author>
    <description></description>
</model>"""

dict  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

def main():
  print(sys.argv)

  if len(sys.argv) < 2:
    sys.exit('Missing output directory.')
  
  outdir = sys.argv[1]

  for i in range(10):
      img = cv2.aruco.drawMarker(dict, i, 1000)
      model_dir = os.path.join(outdir,'aruco_marker_{}'.format(i))
      texture_dir = os.path.join(model_dir, 'materials', 'textures')
      script_dir = os.path.join(model_dir, 'materials', 'scripts')

      # write texture
      os.makedirs(texture_dir, exist_ok=True)
      cv2.imwrite(os.path.join(texture_dir, 'aruco_marker_{}.png'.format(i)), img)

      # write script
      os.makedirs(script_dir, exist_ok=True)
      with open(os.path.join(script_dir, 'aruco_marker_{}.material'.format(i)),'w') as f:
          f.write(script_template.format(i))

      # write model
      with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
          f.write(model_template.format(i))
      
      # write config
      with open(os.path.join(model_dir, 'model.config'), 'w') as f:
          f.write(config_template.format(i))

  print('Done')