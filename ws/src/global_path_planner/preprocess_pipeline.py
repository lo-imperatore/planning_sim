import argparse
import os
from typing import Tuple

import open3d as o3d

from detect_obstacle import identify_and_remove_ground
from identify_ground import identify_ground_plane


def run_pipeline(
    input_ply: str,
    output_dir: str,
) -> Tuple[str, str]:
    """
    Simplified pipeline: identify ground and obstacles only.
    No downsampling, no mapping/height-based filtering.
    
    Returns:
        (cfree_map_ply, cobs_map_ply) - navigable and obstacle maps
    """
    os.makedirs(output_dir, exist_ok=True)
    base_name = os.path.splitext(os.path.basename(input_ply))[0]
    output_prefix = os.path.join(output_dir, base_name)

    print("=" * 60)
    print("STEP 1: IDENTIFY GROUND AND OBSTACLES")
    print("=" * 60)
    
    # Get ground and obstacles
    ground_down, obstacle_down, ground_full, obstacle_full, _, _ = identify_and_remove_ground(
        ply_file_path=input_ply,
        save_obstacles=False,
        output_prefix=output_prefix,
    )
    
    # Save as cfree (navigable/walkable) and cobs (obstacles)
    cfree_map = os.path.join(output_dir, f"cfree_map_{base_name}.ply")
    cobs_map = os.path.join(output_dir, f"cobs_map_{base_name}.ply")
    
    o3d.io.write_point_cloud(cfree_map, ground_full)
    o3d.io.write_point_cloud(cobs_map, obstacle_full)
    
    print(f"\n✓ Cfree (ground/navigable): {len(ground_full.points)} points")
    print(f"✓ Cobs (obstacles): {len(obstacle_full.points)} points")
    print(f"\nSaved:")
    print(f"  - {cfree_map}")
    print(f"  - {cobs_map}")
    
    return cfree_map, cobs_map


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simple pipeline: identify ground as cfree, obstacles as cobs."
    )
    parser.add_argument("input_ply", help="Input PLY file path")
    parser.add_argument("--output-dir", default="output", help="Directory for outputs")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    cfree_map, cobs_map = run_pipeline(
        input_ply=args.input_ply,
        output_dir=args.output_dir,
    )

    print(f"\nCfree map: {cfree_map}")
    print(f"Cobs map: {cobs_map}")


if __name__ == "__main__":
    main()
