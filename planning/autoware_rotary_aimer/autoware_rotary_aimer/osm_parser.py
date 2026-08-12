# Copyright 2024 azzamwildan462
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Minimal OSM parser for tf_pedestrian traffic lights.

Copied (with permission) from autoware_pedestrian_light_detector/roi_projector.py
to avoid cross-package coupling.  Only the Light namedtuple and
parse_tf_pedestrian_lights() are included.

Public API
----------
parse_tf_pedestrian_lights(osm_path) -> list[Light]

Light namedtuple:
  .way_id   str              -- OSM way ID string
  .points   list[(x,y,z)]   -- node coordinates in map frame (local_x/local_y/ele)
  .midpoint (x,y,z)         -- centroid of all points
"""

import xml.etree.ElementTree as ET
from typing import Dict, List, NamedTuple, Tuple


class Light(NamedTuple):
    """Single pedestrian traffic light parsed from the lanelet2 OSM map."""

    way_id: str
    points: List[Tuple[float, float, float]]
    midpoint: Tuple[float, float, float]


def parse_tf_pedestrian_lights(osm_path: str) -> List[Light]:
    """
    Parse a lanelet2 OSM file and return all tf_pedestrian=true ways as Lights.

    Node coordinates are read from local_x / local_y / ele tags (map frame).
    No lanelet2 Python bindings required — pure xml.etree.

    Parameters
    ----------
    osm_path : str
        Absolute path to the .osm file.

    Returns
    -------
    list[Light]
        Empty list if no tf_pedestrian ways are found.

    Raises
    ------
    IOError
        If the file cannot be parsed.
    """
    try:
        tree = ET.parse(osm_path)
    except Exception as exc:
        raise IOError(f'Cannot parse OSM file {osm_path}: {exc}') from exc

    root = tree.getroot()

    # Build dict: node_id -> (x, y, z) for all nodes with local_x/local_y
    node_coords: Dict[str, Tuple[float, float, float]] = {}
    for nd in root.findall('node'):
        nid = nd.get('id', '')
        tags = {t.get('k', ''): t.get('v', '') for t in nd.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            try:
                x = float(tags['local_x'])
                y = float(tags['local_y'])
                z = float(tags.get('ele', '0.0'))
                node_coords[nid] = (x, y, z)
            except ValueError:
                pass

    # Collect ways tagged tf_pedestrian="true"
    lights: List[Light] = []
    for way in root.findall('way'):
        wid = way.get('id', '')
        tags = {t.get('k', ''): t.get('v', '') for t in way.findall('tag')}

        if tags.get('tf_pedestrian') != 'true':
            continue

        points: List[Tuple[float, float, float]] = []
        for nd_ref in way.findall('nd'):
            ref = nd_ref.get('ref', '')
            if ref in node_coords:
                points.append(node_coords[ref])

        if not points:
            continue

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        mid = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))

        lights.append(Light(way_id=wid, points=points, midpoint=mid))

    return lights
