import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))
from pkg.geometry.geometry import *

def add_panda_cam(gscene, tool_link, theta):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="panda_cam_mount_vis", link_name=tool_link, dims=(0.1,0.1,0.1), 
                       center=(0,0,0), rpy=(0,0,theta), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                      uri="package://my_mesh/meshes/stl/cam_mount_v3_res.STL")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_cam_mount_col", link_name=tool_link, dims=(0.061,0.061,0.06), 
                       center=(0.0,0.08,-0.015), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_cam_mount_vis")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_cam_body", link_name=tool_link, dims=(0.061,0.061,0.026), 
                       center=(0.0,0.12,-0.015), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       parent="panda_cam_mount_vis")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_cam_body_col", link_name=tool_link, dims=(0.081,0.081,0.046), 
                       center=(0.0,0.0,0.0), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_cam_body")
    
def add_panda_brush(gscene, tool_link, theta, brush_name, offset=(0,0,0.011), tool_dims=(0.09,0.175,0.05), col_margin=0.01):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="panda_tool_vis", link_name=tool_link, dims=(0.1,0.1,0.1), 
                       center=offset, rpy=(0,0,theta-np.pi/2), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/WipingTool_res.STL")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_tool_mount_col", link_name=tool_link, dims=(0.08,0.08,0.03), 
                       center=(0.0,0.0,0.0), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_root_col", link_name=tool_link, dims=(0.05,0.05,0.04), 
                       center=(0,0.0,0.02), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_rod_col", link_name=tool_link, dims=(0.05,0.05,0.25), 
                       center=(0.088,0.0,0.11), rpy=(0,np.pi/4,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_rod_col", link_name=tool_link, dims=(0.05,0.05,0.25), 
                       center=(0.088,0.0,0.11), rpy=(0,np.pi/4,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=brush_name, link_name=tool_link, dims=tool_dims, 
                       center=(0.22,0.0,0.192), rpy=(0,-np.pi/2,0), display=True, color=(0.8,0.8,0.0,0.9), collision=False, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name=brush_name+"_col", link_name=tool_link, dims=np.add(tool_dims, (col_margin,col_margin,0)),
                       center=(0, 0, col_margin), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent=brush_name)
    return brush_face