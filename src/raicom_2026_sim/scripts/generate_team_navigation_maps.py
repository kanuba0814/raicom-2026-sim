#!/usr/bin/env python3
"""Derive SIM_ONLY team navigation masks from the Gazebo field texture."""

from pathlib import Path

from PIL import Image


MAP_ORIGIN = (-8.0, -8.0)
MAP_RESOLUTION = 0.05
TEXTURE_WORLD_BOUNDS = (-0.526491, 6.628509, -0.303290, 3.346710)
BLUE_RGB = (25, 184, 241)
RED_RGB = (212, 58, 75)
COLOR_TOLERANCE = 70


def color_distance_squared(pixel, reference):
    return sum((int(channel) - target) ** 2 for channel, target in zip(pixel, reference))


def texture_pixel(texture, world_x, world_y):
    minimum_x, maximum_x, minimum_y, maximum_y = TEXTURE_WORLD_BOUNDS
    normalized_x = (world_x - minimum_x) / (maximum_x - minimum_x)
    normalized_y = (maximum_y - world_y) / (maximum_y - minimum_y)
    column = round(normalized_x * (texture.width - 1))
    row = round(normalized_y * (texture.height - 1))
    if column < 0 or row < 0 or column >= texture.width or row >= texture.height:
        return None
    return texture.getpixel((column, row))


def is_zone(pixel, zone_color):
    return (
        pixel is not None
        and color_distance_squared(pixel, zone_color) <= COLOR_TOLERANCE ** 2
    )


def generate(source_map, texture, output, forbidden_color):
    occupancy = Image.open(source_map).convert("L")
    field_texture = Image.open(texture).convert("RGB")
    output_map = occupancy.copy()
    pixels = output_map.load()
    forbidden_cells = 0

    for row in range(output_map.height):
        world_y = MAP_ORIGIN[1] + (output_map.height - row - 0.5) * MAP_RESOLUTION
        for column in range(output_map.width):
            world_x = MAP_ORIGIN[0] + (column + 0.5) * MAP_RESOLUTION
            if is_zone(texture_pixel(field_texture, world_x, world_y), forbidden_color):
                pixels[column, row] = 0
                forbidden_cells += 1

    output_map.save(output)
    return forbidden_cells


def write_boundary_only_mask(source_map, team_map, output):
    """Write occupied forbidden cells over an otherwise-free navigation mask."""
    source = Image.open(source_map).convert("L")
    team = Image.open(team_map).convert("L")
    mask = Image.new("L", source.size, color=254)
    source_pixels, team_pixels, mask_pixels = source.load(), team.load(), mask.load()
    forbidden_cells = 0
    for row in range(source.height):
        for column in range(source.width):
            if team_pixels[column, row] == 0 and source_pixels[column, row] != 0:
                mask_pixels[column, row] = 0
                forbidden_cells += 1
    mask.save(output)
    return forbidden_cells


def write_yaml(path, image_name):
    path.write_text(
        "image: {}\n"
        "resolution: 0.050000\n"
        "origin: [-8.000000, -8.000000, 0.000000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n".format(image_name),
        encoding="utf-8",
    )


def main():
    package = Path(__file__).resolve().parents[1]
    source_map = package / "maps" / "red_blue_boundary_sim.pgm"
    texture = package / "meshes" / "industrial_group_map" / "map_icon.png"
    outputs = {
        "blue": RED_RGB,
        "red": BLUE_RGB,
    }
    for team, forbidden_color in outputs.items():
        pgm = package / "maps" / "{}_team_navigation_sim.pgm".format(team)
        yaml_path = pgm.with_suffix(".yaml")
        cells = generate(source_map, texture, pgm, forbidden_color)
        write_yaml(yaml_path, pgm.name)
        boundary_mask = package / "maps" / "{}_team_boundary_mask_only_sim.pgm".format(team)
        boundary_cells = write_boundary_only_mask(source_map, pgm, boundary_mask)
        write_yaml(boundary_mask.with_suffix(".yaml"), boundary_mask.name)
        print("{}: {} forbidden cells -> {}; {} boundary-only cells -> {}".format(
            team, cells, pgm, boundary_cells, boundary_mask))


if __name__ == "__main__":
    main()
