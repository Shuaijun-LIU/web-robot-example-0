#!/usr/bin/env python3
"""Convert selected RoboTwin GLB tools into centered MuJoCo-compatible OBJ files.

The converter intentionally uses only the Python standard library so the asset
build remains reproducible on machines without Blender or trimesh.
"""

from pathlib import Path
import json
import struct
import sys


TOOLS = {
    "robotwin-screwdriver.obj": "032_screwdriver/visual/base0.glb",
    "robotwin-drill.obj": "030_drill/visual/base6.glb",
    "robotwin-hammer.obj": "020_hammer/visual/base0.glb",
}

COMPONENTS = {
    5120: ("b", 1),
    5121: ("B", 1),
    5122: ("h", 2),
    5123: ("H", 2),
    5125: ("I", 4),
    5126: ("f", 4),
}
WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def multiply(a, b):
    return [[sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4)] for row in range(4)]


def node_matrix(node):
    if "matrix" in node:
        values = node["matrix"]
        return [[values[column * 4 + row] for column in range(4)] for row in range(4)]
    tx, ty, tz = node.get("translation", [0, 0, 0])
    sx, sy, sz = node.get("scale", [1, 1, 1])
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    rotation = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
        [0, 0, 0, 1],
    ]
    scale = [[sx, 0, 0, 0], [0, sy, 0, 0], [0, 0, sz, 0], [0, 0, 0, 1]]
    translation = [[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0, 0, 0, 1]]
    return multiply(translation, multiply(rotation, scale))


def transform_point(matrix, point):
    vector = [point[0], point[1], point[2], 1]
    return [sum(matrix[row][column] * vector[column] for column in range(4)) for row in range(3)]


def read_glb(path):
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2 or length != len(data):
        raise ValueError(f"Unsupported GLB header in {path}")
    offset = 12
    document = None
    binary = None
    while offset < length:
        chunk_length, chunk_type = struct.unpack_from("<I4s", data, offset)
        offset += 8
        chunk = data[offset:offset + chunk_length]
        offset += chunk_length
        if chunk_type == b"JSON":
            document = json.loads(chunk.decode("utf-8"))
        elif chunk_type == b"BIN\x00":
            binary = chunk
    if document is None or binary is None:
        raise ValueError(f"GLB lacks JSON or BIN chunk: {path}")
    return document, binary


def read_accessor(document, binary, accessor_index):
    accessor = document["accessors"][accessor_index]
    view = document["bufferViews"][accessor["bufferView"]]
    if accessor.get("sparse"):
        raise ValueError("Sparse GLB accessors are not supported")
    format_code, component_size = COMPONENTS[accessor["componentType"]]
    width = WIDTHS[accessor["type"]]
    element_size = component_size * width
    stride = view.get("byteStride", element_size)
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    unpack = struct.Struct("<" + format_code * width).unpack_from
    return [unpack(binary, start + index * stride) for index in range(accessor["count"])]


def extract_mesh(path):
    document, binary = read_glb(path)
    nodes = document.get("nodes", [])
    parents = {}
    for parent_index, node in enumerate(nodes):
        for child in node.get("children", []):
            parents[child] = parent_index

    world_cache = {}

    def world_matrix(index):
        if index not in world_cache:
            local = node_matrix(nodes[index])
            world_cache[index] = multiply(world_matrix(parents[index]), local) if index in parents else local
        return world_cache[index]

    vertices = []
    texcoords = []
    faces = []
    for node_index, node in enumerate(nodes):
        if "mesh" not in node:
            continue
        transform = world_matrix(node_index)
        for primitive in document["meshes"][node["mesh"]]["primitives"]:
            if primitive.get("mode", 4) != 4:
                raise ValueError(f"Only GL_TRIANGLES primitives are supported in {path}")
            positions = read_accessor(document, binary, primitive["attributes"]["POSITION"])
            source_texcoords = read_accessor(
                document,
                binary,
                primitive["attributes"]["TEXCOORD_0"],
            ) if "TEXCOORD_0" in primitive["attributes"] else [(0, 0)] * len(positions)
            base = len(vertices)
            vertices.extend(transform_point(transform, point) for point in positions)
            texcoords.extend((uv[0], 1 - uv[1]) for uv in source_texcoords)
            if "indices" in primitive:
                indices = [item[0] for item in read_accessor(document, binary, primitive["indices"])]
            else:
                indices = list(range(len(positions)))
            faces.extend(tuple(base + indices[index + axis] for axis in range(3)) for index in range(0, len(indices), 3))
    material = document["materials"][0]["pbrMetallicRoughness"]
    texture_index = material["baseColorTexture"]["index"]
    image_index = document["textures"][texture_index]["source"]
    image = document["images"][image_index]
    view = document["bufferViews"][image["bufferView"]]
    start = view.get("byteOffset", 0)
    image_bytes = binary[start:start + view["byteLength"]]
    return vertices, texcoords, faces, image_bytes, image["mimeType"]


def normalize(vertices):
    # RoboTwin's selected tools are authored lengthwise on +Y. Rotate that axis
    # onto local +X while preserving a right-handed coordinate frame.
    rotated = [[vertex[1], -vertex[0], vertex[2]] for vertex in vertices]
    minimum = [min(vertex[axis] for vertex in rotated) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in rotated) for axis in range(3)]
    center = [(minimum[axis] + maximum[axis]) / 2 for axis in range(3)]
    centered = [[vertex[axis] - center[axis] for axis in range(3)] for vertex in rotated]
    return centered, [maximum[axis] - minimum[axis] for axis in range(3)]


def write_obj(path, vertices, texcoords, faces):
    lines = ["# Converted from RoboTwin GLB for MuJoCo web loading"]
    lines.extend("v " + " ".join(f"{value:.8g}" for value in vertex) for vertex in vertices)
    lines.extend("vt " + " ".join(f"{value:.8g}" for value in texcoord) for texcoord in texcoords)
    lines.extend(
        "f " + " ".join(f"{index + 1}/{index + 1}" for index in face)
        for face in faces
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: convert-robotwin-tools.py <robotwin-objects-dir> <output-dir>")
    source_root = Path(sys.argv[1]).resolve()
    output_root = Path(sys.argv[2]).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    for output_name, relative_source in TOOLS.items():
        vertices, texcoords, faces, image_bytes, mime_type = extract_mesh(source_root / relative_source)
        vertices, extents = normalize(vertices)
        write_obj(output_root / output_name, vertices, texcoords, faces)
        extension = {"image/png": ".png", "image/jpeg": ".jpg"}.get(mime_type)
        if extension is None:
            raise ValueError(f"Unsupported base-color texture type: {mime_type}")
        (output_root / Path(output_name).with_suffix(extension)).write_bytes(image_bytes)
        print(f"{output_name}: {len(vertices)} vertices, {len(faces)} faces, extents={extents}")


if __name__ == "__main__":
    main()
