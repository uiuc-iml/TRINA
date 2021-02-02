# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
#SPHINXBUILD   = sphinx-build
SPHINXBUILD   = python3 -msphinx
SPHINXAPIDOC  = sphinx-apidoc
SOURCEDIR     = source
MODULEDIR     = ../trina
BUILDDIR      = build
PIP = python3 -m pip

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help pip-packages Makefile

pip-packages:
	@$(PIP) install sphinx sphinx-automodapi sphinxcontrib-napoleon sphinx_rtd_theme

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXAPIDOC) -f -o "$(SOURCEDIR)" $(MODULEDIR)
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)