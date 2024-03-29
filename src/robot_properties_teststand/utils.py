try:
    # use standard Python importlib if available (>Python3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources

import glob
import sys
from os import path, walk, mkdir, access, X_OK, environ, pathsep
import subprocess

from xacro import process_file, open_output
from xacro.color import warning, error, message
from xacro.xmlutils import *
from xacro.cli import process_args

_basestr = str
unicode = str
encoding = {}

def find_paths(robot_name, robot_family="teststand"):
    with importlib_resources.path(__package__, "utils.py") as p:
            package_dir = p.parent.absolute()
    
    resources_dir = package_dir/("robot_properties_" + robot_family)
    dgm_yaml_path = resources_dir/"dynamic_graph_manager"/("dgm_parameters_" + robot_name + ".yaml")
    urdf_path = resources_dir/(robot_name + ".urdf")
    urdf_no_prismatic = resources_dir/(robot_name + "_no_prismatic.urdf")
    srdf_path = resources_dir/"srdf"/(robot_family + ".srdf")
    ctrl_path = resources_dir/"impedance_ctrl.yaml"

    if not urdf_path.exists() or not urdf_no_prismatic.exists():
        build_xacro_files(resources_dir)

    paths = {"package":str(package_dir),
             "resources":str(resources_dir),
             "dgm_yaml":str(dgm_yaml_path),
             "srdf":str(srdf_path),
             "urdf":str(urdf_path),
             "urdf_no_prismatic": str(urdf_no_prismatic),
             "imp_ctrl_yaml":str(ctrl_path)}

    return paths


def build_xacro_files(resources_dir):
    """ Look for the xacro files and build them in the build folder. """

    build_folder = resources_dir
    xacro_files = []
    for (root, _, files) in walk(path.join(resources_dir, "xacro")):
        for afile in files:
            if afile.endswith(".urdf.xacro"):
                xacro_files.append(path.join(root, afile))

    if not path.exists(build_folder):
        mkdir(build_folder)

    for xacro_file in xacro_files:
        for xacro_file in xacro_files:
            # Generated file name
            generated_urdf_path = path.join(
                build_folder, path.basename(path.splitext(xacro_file)[0])
            )
            # Call xacro in bash
            # bash_command = ["xacro", xacro_file, "-o", generated_urdf_path]
            # process = subprocess.Popen(bash_command, stdout=subprocess.PIPE)
            # process.communicate()
            build_single_xacro_file(xacro_file, generated_urdf_path)


def build_single_xacro_file(input_path, output_path):
    try:
        # open and process file
        doc = process_file(input_path)
        # open the output file
        out = open_output(output_path)

    except xml.parsers.expat.ExpatError as e:
        error("XML parsing error: %s" % unicode(e), alt_text=None)
        sys.exit(2) 

    except Exception as e:
        msg = unicode(e)
        if not msg: msg = repr(e)
        error(msg)
        sys.exit(2)  # gracefully exit with error condition

    # write output
    out.write(doc.toprettyxml(indent='  ', **encoding))
    # only close output file, but not stdout
    out.close()



        