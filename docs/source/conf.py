# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
sys.path.insert(0, os.path.abspath('../../scripts'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Research Track I - Second Assignment Part I'
copyright = '2025, Rubin Khadka Chhetri'
author = 'Rubin Khadka Chhetri'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'sphinx_copybutton',
]

templates_path = ['_templates']
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# Extension configuration

# -- Options for intersphinx extension ---------------------------------------
intersphinx_mapping = {'python': ('https://docs.python.org/3', None)}

# -- Options for todo extension ----------------------------------------------
# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True