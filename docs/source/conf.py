# Configuration file for the Sphinx documentation builder.

import os
import sys

# -- Project information -----------------------------------------------------

project = 'ROS Central Registry'
copyright = '2025, Open Source Robotics Foundation, Inc.'
author = 'Open Source Robotics Foundation, Inc.'
release = '0.0.1'

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx_tabs.tabs',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------

html_theme = 'furo'
html_static_path = ['_static']
html_css_files = [
    'custom.css',
]
html_logo = "rcr_logo.png"

html_theme_options = {
    "sidebar_hide_name": False,
}
