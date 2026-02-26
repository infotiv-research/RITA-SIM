#!/usr/bin/env python3
"""Executable entrypoint for the modular target-object pick-and-place node."""

import rclpy

from pick_and_place.workflow import PickAndPlaceTargetObject


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTargetObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
