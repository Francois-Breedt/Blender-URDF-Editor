"""
urdf_parser.py  —  Pure-Python URDF XML parser, no Blender dependency.
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional
import os


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Pose:
    xyz: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)


@dataclass
class Geometry:
    geo_type: str = "mesh"          # mesh | box | cylinder | sphere
    mesh_filename: str = ""
    scale: tuple = (1.0, 1.0, 1.0)
    size: tuple = (0.0, 0.0, 0.0)  # box
    radius: float = 0.0             # cylinder / sphere
    length: float = 0.0             # cylinder
    sphere_radius: float = 0.0


@dataclass
class Visual:
    name: str = ""
    origin: Pose = field(default_factory=Pose)
    geometry: Optional[Geometry] = None
    material_name: str = ""
    color_rgba: tuple = (0.7, 0.7, 0.7, 1.0)


@dataclass
class Collision:
    name: str = ""
    origin: Pose = field(default_factory=Pose)
    geometry: Optional[Geometry] = None


@dataclass
class URDFLink:
    name: str = ""
    visuals: list = field(default_factory=list)
    collisions: list = field(default_factory=list)


@dataclass
class JointDynamics:
    damping: float = 0.0
    friction: float = 0.0


@dataclass
class JointLimit:
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class JointMimic:
    joint: str = ""
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class URDFJoint:
    name: str = ""
    joint_type: str = "fixed"
    parent: str = ""
    child: str = ""
    origin: Pose = field(default_factory=Pose)
    axis: tuple = (1.0, 0.0, 0.0)
    limit: Optional[JointLimit] = None
    dynamics: Optional[JointDynamics] = None
    mimic: Optional[JointMimic] = None


@dataclass
class URDFRobot:
    name: str = ""
    links: dict = field(default_factory=dict)   # name -> URDFLink
    joints: dict = field(default_factory=dict)  # name -> URDFJoint
    base_dir: str = ""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_xyz(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return default
    vals = [float(v) for v in text.strip().split()]
    return tuple(vals[:3]) if len(vals) >= 3 else default


def _parse_pose(elem) -> Pose:
    if elem is None:
        return Pose()
    return Pose(
        xyz=_parse_xyz(elem.get("xyz", "0 0 0")),
        rpy=_parse_xyz(elem.get("rpy", "0 0 0")),
    )


def _resolve_package(filename: str, base_dir: str) -> str:
    """
    Resolve a mesh filename to an absolute path.

    Handles:
      1. package://<pkg_name>/<rel_path>  — standard ROS convention
      2. Relative path                    — resolved from base_dir
      3. Absolute path                    — returned as-is

    For (1) the package root is found by trying multiple candidate directories
    in priority order, returning the first one where the file actually exists.
    """
    if filename.startswith("package://"):
        without_scheme = filename[len("package://"):]
        slash = without_scheme.find("/")
        if slash != -1:
            pkg_name = without_scheme[:slash]
            rel_path = without_scheme[slash + 1:]
        else:
            pkg_name = without_scheme
            rel_path = ""

        candidates = []

        # Walk up directory tree looking for a folder named pkg_name
        search = base_dir
        for _ in range(8):
            if os.path.basename(search) == pkg_name:
                candidates.append(search)
                break
            parent = os.path.dirname(search)
            if parent == search:
                break
            search = parent

        # base_dir itself, its parent, and grandparent as fallbacks
        candidates.append(base_dir)
        candidates.append(os.path.dirname(base_dir))
        candidates.append(os.path.dirname(os.path.dirname(base_dir)))

        for root in candidates:
            resolved = os.path.normpath(os.path.join(root, rel_path))
            if os.path.exists(resolved):
                return resolved

        # Nothing found — return parent-of-base_dir guess for a useful error
        return os.path.normpath(os.path.join(os.path.dirname(base_dir), rel_path))

    if not os.path.isabs(filename):
        return os.path.normpath(os.path.join(base_dir, filename))

    return filename


def _parse_geometry(geo_elem, base_dir: str) -> Optional[Geometry]:
    if geo_elem is None:
        return None

    mesh = geo_elem.find("mesh")
    if mesh is not None:
        raw = mesh.get("filename", "")
        filename = _resolve_package(raw, base_dir)
        print(f"[URDF] Resolved mesh: {raw!r}  →  {filename!r}  exists={os.path.exists(filename)}")
        scale_str = mesh.get("scale", "1 1 1")
        scale = _parse_xyz(scale_str, (1.0, 1.0, 1.0))
        return Geometry(geo_type="mesh", mesh_filename=filename, scale=scale)

    box = geo_elem.find("box")
    if box is not None:
        size = _parse_xyz(box.get("size", "0 0 0"))
        return Geometry(geo_type="box", size=size)

    cyl = geo_elem.find("cylinder")
    if cyl is not None:
        return Geometry(
            geo_type="cylinder",
            radius=float(cyl.get("radius", 0)),
            length=float(cyl.get("length", 0)),
        )

    sph = geo_elem.find("sphere")
    if sph is not None:
        return Geometry(geo_type="sphere", sphere_radius=float(sph.get("radius", 0)))

    return None


def _parse_material(vis_elem, materials_global: dict):
    mat_elem = vis_elem.find("material")
    if mat_elem is None:
        return ("", (0.7, 0.7, 0.7, 1.0))
    name = mat_elem.get("name", "")
    color_elem = mat_elem.find("color")
    if color_elem is not None:
        rgba_str = color_elem.get("rgba", "0.7 0.7 0.7 1.0")
        rgba = tuple(float(v) for v in rgba_str.split())
        return (name, rgba)
    if name and name in materials_global:
        return (name, materials_global[name])
    return (name, (0.7, 0.7, 0.7, 1.0))


# ---------------------------------------------------------------------------
# Main parser
# ---------------------------------------------------------------------------

def parse_urdf(filepath: str) -> URDFRobot:
    tree = ET.parse(filepath)
    root = tree.getroot()
    base_dir = os.path.dirname(os.path.abspath(filepath))

    robot = URDFRobot(name=root.get("name", "robot"), base_dir=base_dir)

    # Pre-pass: global material definitions
    global_materials = {}
    for mat_elem in root.findall("material"):
        mname = mat_elem.get("name", "")
        color_elem = mat_elem.find("color")
        if color_elem is not None:
            rgba = tuple(float(v) for v in color_elem.get("rgba", "0.7 0.7 0.7 1.0").split())
            global_materials[mname] = rgba

    # Parse links
    for link_elem in root.findall("link"):
        link = URDFLink(name=link_elem.get("name", ""))

        for vis_elem in link_elem.findall("visual"):
            v = Visual(
                name=vis_elem.get("name", ""),
                origin=_parse_pose(vis_elem.find("origin")),
                geometry=_parse_geometry(vis_elem.find("geometry"), base_dir),
            )
            v.material_name, v.color_rgba = _parse_material(vis_elem, global_materials)
            link.visuals.append(v)

        for col_elem in link_elem.findall("collision"):
            c = Collision(
                name=col_elem.get("name", ""),
                origin=_parse_pose(col_elem.find("origin")),
                geometry=_parse_geometry(col_elem.find("geometry"), base_dir),
            )
            link.collisions.append(c)

        robot.links[link.name] = link

    # Parse joints
    for jnt_elem in root.findall("joint"):
        j = URDFJoint(
            name=jnt_elem.get("name", ""),
            joint_type=jnt_elem.get("type", "fixed"),
        )

        parent_el = jnt_elem.find("parent")
        child_el  = jnt_elem.find("child")
        j.parent = parent_el.get("link", "") if parent_el is not None else ""
        j.child  = child_el.get("link",  "") if child_el  is not None else ""
        j.origin = _parse_pose(jnt_elem.find("origin"))

        axis_el = jnt_elem.find("axis")
        if axis_el is not None:
            j.axis = _parse_xyz(axis_el.get("xyz", "1 0 0"))

        lim_el = jnt_elem.find("limit")
        if lim_el is not None:
            j.limit = JointLimit(
                lower   =float(lim_el.get("lower",    0)),
                upper   =float(lim_el.get("upper",    0)),
                effort  =float(lim_el.get("effort",   0)),
                velocity=float(lim_el.get("velocity", 0)),
            )

        dyn_el = jnt_elem.find("dynamics")
        if dyn_el is not None:
            j.dynamics = JointDynamics(
                damping =float(dyn_el.get("damping",  0)),
                friction=float(dyn_el.get("friction", 0)),
            )

        mimic_el = jnt_elem.find("mimic")
        if mimic_el is not None:
            j.mimic = JointMimic(
                joint      =mimic_el.get("joint",       ""),
                multiplier =float(mimic_el.get("multiplier", 1.0)),
                offset     =float(mimic_el.get("offset",     0.0)),
            )

        robot.joints[j.name] = j

    return robot
