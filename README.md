# Blender URDF Editor
An addon that enables editing of URDF files inside Blender.

## Requirements
Blender 5.0 or newer

## Installation
- Download the .zip file.
<img width="421" height="384" alt="image" src="https://github.com/user-attachments/assets/3de51355-b0a2-451c-89f7-792f5e303718" />

- In BLender, go to **Edit -> Preferences -> Add-ons**, and select **Install from Disk** in the drop down menu.
<img width="456" height="194" alt="image" src="https://github.com/user-attachments/assets/342d6e0a-509f-4440-9686-c56c37e6fccb" />

- Navigate to the downloaded .zip file, and click the button to install it from disk.

## Usage
- Press N to open the side panel, and select the URDF tab
<img width="470" height="320" alt="image" src="https://github.com/user-attachments/assets/73fb421b-bb31-4bac-9fc4-1e6ea49601bf" />

- Click the **Import URDF** button and select the URDF file to import. Assuming a typical folder structure, the meshes should be imported correctly. This folder structure works:

  - robot_description/
    - urdf/
      - robot.urdf
    - meshes/
      - mesh_1.stl
      - ...

- You should be able to see the robot in the 3D viewport. Importantly, since the main goal is to edit the collision geometry, any collision geometry in the URDF is ignored, and the visual geometry is simply duplicated. The outliner should look something like follows
<img width="455" height="162" alt="image" src="https://github.com/user-attachments/assets/aab467e1-f284-4b45-a040-1624e995ce09" />

- The **Joint Values** section also contains all of the joints and their limits, which can be edited. It is recommended not to move anything directly in the viewport, and to rather use the given joint value sliders.
<img width="286" height="414" alt="image" src="https://github.com/user-attachments/assets/715d6bf7-223f-4218-9f62-a7ddfb734558" />

- The next section allows for easy simplification of geometry. Simply select the mesh you want to simplify, click on one of the modifiers, and apply it.
<img width="306" height="230" alt="image" src="https://github.com/user-attachments/assets/9e589588-9bb1-4939-ab04-c578ff4fd8a2" />

- If **Report Polygon Counts** is clicked, a message will briefly appear at the bottom of the window.
<img width="529" height="36" alt="image" src="https://github.com/user-attachments/assets/f2e58d14-49fe-4887-b68e-ebb88549dd7e" />

- Finally, if you are happy with the result, click on the **Export URDF** button. It will place all of the files in the location of the original URDF file. It is up to you to move all of the mesh files to their correct places. By exporting, **only** the new URDF file and the collision meshes are exported. In the URDF file, only the collision, and joint limit tags are changed. The path references in the URDF file remains unchanged, and the references to the generated collision geometry looks like: original/path/to/meshes/link_name_collision.stl

## Contribution
This is a simple project created to speed up my own workflows. I do not intend to actively update, maintain, or improve it. However, contributions are definitely welcome. Feel free to add features and build on this (very basic) framework. If you would like a feature, but don't know how to add it, feel free to ask. I am more than willing to look into it and add it if I am able to.
