import os
import xml.etree.ElementTree as ET
from glob import glob
from setuptools import setup

# Parse package.xml for metadata
package_xml_path = os.path.join(os.getcwd(), "package.xml")
package_xmldata = ET.parse(package_xml_path)

package_name = package_xmldata.find("name").text
version_str = package_xmldata.find("version").text
description_str = package_xmldata.find("description").text
license_str = package_xmldata.find("license").text
maintainer = package_xmldata.find("maintainer")
maintainer_name_str = maintainer.text
maintainer_email_str = maintainer.attrib["email"]

setup(
    name=package_name,
    version=version_str,
    packages=["btexample", "btexample.nodes"],
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        )
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=maintainer_name_str,
    maintainer_email=maintainer_email_str,
    description=description_str,
    license=license_str,
    tests_require=["pytest", "pytest-cov"],
    entry_points={
        "console_scripts": [
            "btexample_node = btexample.nodes.example:main"
        ],
        # THIS is important for ros_bt_py to discover your custom nodes
        "ros_bt_py.nodes": [
            "DepthSensorNode = btexample.nodes.depth_sensor_node:DepthSensorNode",
            "ThrusterControlNode = btexample.nodes.thruster_control_node:ThrusterControlNode",
        ],
    },
)
