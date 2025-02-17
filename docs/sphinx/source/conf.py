# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import subprocess
from ament_index_python.packages import get_package_share_directory

project = 'lidar_mirror_fov_reshaper'
copyright = '2025, Andreas Loeffler'
author = 'Andreas Loeffler'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'breathe',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.graphviz',
    'sphinx.ext.inheritance_diagram',
]

templates_path = ['_templates']
exclude_patterns = ['*_lidar_mirror_fov_reshaper*']

# latex equation within function descr. to be rendered in the documentation
mathjax3_config = {
    'tex': {'tags': 'ams', 'useLabelIds': True},
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

autosectionlabel_prefix_document = True

subprocess.call('make clean', shell=True)
subprocess.call('cd ../../doxygen ; doxygen', shell=True)

breathe_projects = {"lidar_mirror_fov_reshaper_runtime": "../../doxygen/build/xml/",
                    "lidar_mirror_fov_reshaper_calibration": "../../doxygen/build/xml/",
                    "lidar_mirror_fov_reshaper_transformation": "../../doxygen/build/xml/"}
breathe_default_project = "lidar_mirror_fov_reshaper_runtime"

