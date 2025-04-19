#!/usr/bin/env python3
# block_diagram_builder.py

# -*- coding: utf-8 -*-
"""
block_diagram_builder.py

Module for generating functional block diagrams using graphviz based on
a YAML configuration file. Supports hierarchical clusters (subgraphs),
node/edge attributes, rank constraints, and multiple output formats.

Author: Brighton Sikarskie
Date: 2024-02-10

Enhancements:
 - Uses Graphviz's unflatten tool to dynamically adjust layout,
   preventing overly zoomed-out diagrams.
 - Removes any fixed 'size' or 'ratio' from the YAML to let code handle layout.
 - Unbinds final_dot.filename after loading from unflattened file to avoid
   the "No such file or directory" error when rendering to a different filename.
"""

import graphviz
import yaml
import logging
import subprocess
import os
from typing import Dict, Any, List, Optional

logger = logging.getLogger(__name__)

def create_clusters(parent: graphviz.Digraph, clusters_config: List[Dict[str, Any]]) -> None:
    """
    Recursively create clusters (subgraphs) as defined in the 'clusters' section of the YAML.

    Args:
        parent (graphviz.Digraph): The parent graph or subgraph in which to create clusters.
        clusters_config (List[Dict[str, Any]]): A list of cluster definitions, each containing:
            - name (str): The name/ID of the cluster.
            - attributes (dict): Graphviz subgraph attributes (label, style, color, etc.).
            - subgraphs (List[Dict[str, Any]]): Nested cluster definitions (optional).
            - nodes (List[Dict[str, Any]]): Nodes to create inside this cluster (optional).
    """
    for cluster in clusters_config:
        cluster_name = cluster.get('name')
        if not cluster_name:
            logger.warning("Encountered a cluster without a 'name' field; skipping.")
            continue

        # Create a subgraph named 'cluster_<name>' to follow Graphviz's cluster naming convention
        subgraph_id = f"cluster_{cluster_name}"
        logger.debug(f"Creating subgraph: {subgraph_id}")

        with parent.subgraph(name=subgraph_id) as sub:
            # Apply the cluster's graph attributes
            attrs = cluster.get('attributes', {})
            sub.attr(**attrs)

            # Recursively handle nested subgraphs if present
            nested_subgraphs = cluster.get('subgraphs', [])
            if nested_subgraphs:
                logger.debug(f"Cluster '{cluster_name}' has {len(nested_subgraphs)} nested subgraphs.")
                create_clusters(sub, nested_subgraphs)

            # Create nodes inside this cluster
            cluster_nodes = cluster.get('nodes', [])
            for node in cluster_nodes:
                node_id = node.get('id')
                node_name = node.get('name', '')
                node_attrs = node.get('attributes', {})
                if not node_id:
                    logger.warning(f"Node in cluster '{cluster_name}' has no 'id' field; skipping.")
                    continue
                logger.debug(f"Adding node '{node_id}' to cluster '{cluster_name}'.")
                sub.node(node_id, node_name, **node_attrs)


def run_unflatten(input_gv: str, output_gv: str, max_depth: int = 3) -> None:
    """
    Runs the 'unflatten' tool to improve the layout of a Graphviz .gv file.
    This helps avoid overly wide or zoomed-out diagrams by balancing rank layers.

    Args:
        input_gv (str): Path to the original .gv file.
        output_gv (str): Path to the output .gv file after unflatten.
        max_depth (int): Maximum depth for unflatten layering (-l).
    """
    logger.debug(f"Running unflatten on '{input_gv}' -> '{output_gv}' with max_depth={max_depth}")
    # Requires Graphviz to be installed and 'unflatten' available in PATH
    subprocess.run(
        ["unflatten", f"-l{max_depth}", "-f", "-o", output_gv, input_gv],
        check=True
    )


def create_block_diagram(yaml_file: str, output_file: str = "block_diagram") -> None:
    """
    Create a functional block diagram from YAML configuration using Graphviz.
    Uses 'unflatten' to dynamically adjust layout and avoid excessive zoom-out.

    Args:
        yaml_file (str): Path to YAML file containing block diagram configuration.
        output_file (str): Base name of the output files (without extension).

    Raises:
        Exception: If there is any error in loading or processing the YAML configuration.
    """
    try:
        # Load block diagram configuration from YAML
        with open(yaml_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        logger.info("YAML configuration loaded successfully.")

        # Create a new directed graph with configurable attributes
        dot = graphviz.Digraph(comment=config.get('comment', 'Functional Block Diagram'))
        logger.debug("Initialized main Graphviz Digraph.")

        # Apply global graph attributes from config
        graph_attrs = config.get('graph_attributes', {})
        for key, value in graph_attrs.items():
            if isinstance(value, dict):
                dot.attr(key, **value)
            else:
                dot.attr(key, str(value))
        logger.debug("Applied global graph attributes.")

        # Add title if specified
        if 'title' in config:
            title_attrs = config.get('title_attributes', {
                'labelloc': 't',
                'fontsize': '16',
                'fontname': 'Arial'
            })
            title_text = f"{config['title']}\n{config.get('description', '')}"
            dot.attr('graph', **title_attrs)
            dot.attr('graph', label=title_text)
            logger.debug(f"Applied title attributes and set title to '{title_text}'.")

        # Create the top-level clusters/subgraphs
        clusters_config = config.get('clusters', [])
        if clusters_config:
            logger.debug(f"Found {len(clusters_config)} top-level clusters. Creating subgraphs.")
            create_clusters(dot, clusters_config)
        else:
            logger.debug("No top-level clusters found in the configuration.")

        # Create any top-level nodes (not in a cluster)
        if 'nodes' in config:
            for node in config['nodes']:
                node_id = node.get('id')
                node_name = node.get('name', '')
                node_attrs = node.get('attributes', {})
                if not node_id:
                    logger.warning("A top-level node is missing 'id'; skipping.")
                    continue
                logger.debug(f"Adding top-level node '{node_id}'.")
                dot.node(node_id, node_name, **node_attrs)

        # Add connections (edges) with configurable styling
        for connection in config.get('connections', []):
            edge_attrs = connection.get('attributes', {}).copy()
            from_node = connection.get('from')
            to_node = connection.get('to')
            if not from_node or not to_node:
                logger.warning("A connection is missing 'from' or 'to'; skipping.")
                continue
            label = connection.get('label', '')
            logger.debug(f"Creating edge from '{from_node}' to '{to_node}' with label '{label}'.")
            dot.edge(from_node, to_node, xlabel=label, **edge_attrs)

        # 1) Save the raw .gv file
        gv_file = output_file + ".gv"
        dot.save(gv_file)
        logger.info(f"Saved raw .gv file: {gv_file}")

        # 2) Run unflatten to improve the layout dynamically
        unflat_file = output_file + "_unflat.gv"
        run_unflatten(gv_file, unflat_file, max_depth=3)

        # 3) Load the unflattened .gv file
        final_dot = graphviz.Source.from_file(unflat_file)

        # 4) Set directory and filename for rendering
        # Create a new Source object to avoid file existence checks
        dot_content = final_dot.source
        render_dot = graphviz.Source(dot_content)
        render_dot.directory = "."  # Always render in current directory

        # 5) Render in all requested formats
        output_formats = config.get('output_formats', ['pdf', 'png'])
        for fmt in output_formats:
            render_dot.format = fmt
            outpath = render_dot.render(filename=output_file, cleanup=True)
            logger.info(f"Successfully generated '{outpath}' with unflattened layout.")

        # Clean up intermediate .gv files
        os.remove(gv_file)
        os.remove(unflat_file)
        logger.debug("Cleaned up intermediate .gv files")

    except Exception as e:
        logger.error(f"Error generating block diagram: {e}")
        raise
