"""Optional Blender preview of the generated visual; no simulation behaviour."""
import bpy
from mathutils import Vector
from pathlib import Path

here=Path(__file__).resolve().parent
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
def imported(name, location=(0,0,0)):
    bpy.ops.wm.obj_import(filepath=str(here/'robot'/f'{name}.obj'),forward_axis='Y',up_axis='Z')
    selected=list(bpy.context.selected_objects)
    for ob in selected: ob.location=Vector(ob.location)+Vector(location)
    return selected
imported('body')
imported('wheel',(-.01315,.095,.0115))
imported('wheel',(-.01315,-.095,.0115))
bpy.ops.mesh.primitive_plane_add(size=200, location=(0,0,-.0005))
ground=bpy.context.object;ground.name='Preview_ground'
mat=bpy.data.materials.new('Preview_white');mat.diffuse_color=(.74,.76,.79,1);ground.data.materials.append(mat)
scene=bpy.context.scene
scene.render.engine='CYCLES';scene.cycles.samples=32
scene.cycles.use_denoising=True
scene.render.resolution_x=1400;scene.render.resolution_y=1050;scene.render.resolution_percentage=100
scene.world.color=(.5,.5,.5)
for location,power,size in [((.10,-.12,.35),12,.28),((-.2,.18,.18),7,.22),((.20,.25,.30),9,.20)]:
    bpy.ops.object.light_add(type='AREA',location=location)
    light=bpy.context.object;light.data.energy=power;light.data.shape='DISK';light.data.size=size
    light.rotation_euler=(Vector((0,0,0))-light.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(.21,-.29,.26))
camera=bpy.context.object;camera.rotation_euler=(Vector((.002,0,.009))-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO';camera.data.ortho_scale=.31;camera.data.clip_start=.001;camera.data.clip_end=100
scene.camera=camera
scene.view_settings.view_transform='AgX';scene.view_settings.look='AgX - Medium High Contrast';scene.view_settings.exposure=-1.4
scene.render.image_settings.file_format='PNG'
scene.render.filepath=str(here/'robot'/'preview.png')
bpy.ops.wm.save_as_mainfile(filepath=str(here/'robot'/'preview.blend'))
bpy.ops.render.render(write_still=True)
