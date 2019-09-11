=============
coop_assembly
=============

.. start-badges

.. image:: https://img.shields.io/badge/License-MIT-blue.svg
    :target: https://github.com/stefanaparascho/coop_assembly/blob/master/LICENSE
    :alt: License MIT

.. .. image:: https://travis-ci.org/{{cookiecutter.github_organization}}/{{cookiecutter.project_slug}}.svg?branch=master
..     :target: https://travis-ci.org/{{cookiecutter.github_organization}}/{{cookiecutter.project_slug}}
..     :alt: Travis CI

.. end-badges

.. Write project description

**coop_assembly**: Geometry generation of robotically assembled spatial structures.

Installation
------------

.. Write installation instructions here

::

  git clone https://github.com/createchaos/coop_assembly
  cd coop_assembly
  pip install .
  # try `pip install -e .` if you are developing this package

Working in a conda environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is recommended to set up a conda environment before installing the package, to do this, 
first download `miniconda <https://docs.conda.io/en/latest/miniconda.html>`_, and then run the
following commands in your terminal:

::

    cd test_envs
    conda env create -f coop_assembly_ws.yml

Then, activate the newly created conda environment and install ``coop_assembly``:

::

    conda activate coop_assembly_ws
    # cd to your `coop_assembly` directory
    pip install .

If you've installed the environment following the instructions above,
but want to update the env (e.g. if the ``yml`` file is updated),
run the following command to update:

::

    conda env update -n coop_assembly_ws -f coop_assembly_ws.yml --prune

Examples
--------

``coop_assembly`` can be used directly in Rhino/Grasshopper environment, where
the function calls are handled by ``compas.utilities.xfunc`` or 
``compas.rpc`` (requires ``compas 0.7.0+``)
and visualization is provided by ``compas_ghpython.utilities``.

In order to make ``coop_assembly`` accessible in Rhino/Grasshopper,
we need the run the following command first and **restart Rhino**:

::

    # make compas itself accessible in Rhino
    python -m compas_rhino.install

    # make coop_assembly accessible in Rhino
    python -m compas_rhino.install -p coop_assembly compas_fab roslibpy

And you should be able to see outputs like:

::

   Installing COMPAS packages to Rhino 6.0 IronPython lib:
   IronPython location: C:\Users\<User Name>\AppData\Roaming\McNeel\Rhinoceros\6.0\Plug-ins\IronPython (814d908a-e25c-493d-97e9-ee3861957f49)\settings\lib

   compas               OK
   compas_rhino         OK
   compas_ghpython      OK
   compas_bootstrapper  OK
   coop_assembly        OK

   Completed.

For Rhino 5 users, add ``-v 5.0`` to the command above.

Grasshopper examples can be found in ``examples\gh_interface_scripts``.

Troubleshooting 
---------------

Sometimes things don't go as expected. Here are some of answers to the most common issues you might bump into:

    Q: `conda` commands don't work.

Try running them from the *Conda Prompt*. Depending on how you installed Anaconda, it might not be available by default on the normal Windows command prompt.

    Q: When trying to install the framework in Rhino, it fails indicating the lib folder of IronPython does not exist.

Make sure you have opened Rhino 6 and Grasshopper at least once, so that it finishes setting up all its internal folder structure.

    Q: Error: Microsoft Visual C++ 14.0 is required

Follow the link to install Microsoft Visual C++ 14.0
https://www.scivision.co/python-windows-visual-c++-14-required/

Credits
-------

This package was created by Stefana Parascho <parascho@princeton.edu> `@stefanaparascho <https://github.com/stefanaparascho>`_ 
at the CREATE lab, Princeton University with `collaborators <./AUTHORS.rst>`_.