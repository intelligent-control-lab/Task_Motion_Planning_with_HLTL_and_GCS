import open3d as o3d
import os

# Define the folder path
folder_path = "./"  # Modify this if your STL files are in a different directory

# Loop through all files in the folder
for filename in os.listdir(folder_path):
    if filename.endswith(".stl") or filename.endswith(".STL"):  # Check for STL files
        # Construct full file path
        stl_path = os.path.join(folder_path, filename)
        
        # Read the STL mesh
        mesh = o3d.io.read_triangle_mesh(stl_path)
        
        # Construct OBJ file path (replace .stl with .obj)
        obj_filename = filename.replace(".stl", ".obj").replace(".STL", ".obj")
        obj_path = os.path.join(folder_path, obj_filename)
        
        # Write the mesh to an OBJ file
        o3d.io.write_triangle_mesh(obj_path, mesh)
        print(f"Converted {filename} to {obj_filename}")

print("All STL files have been converted to OBJ files.")