import os
from glob import glob

from setuptools import setup

package_name = "kmu26_auv_web_gui"


def data_files_for(directory):
    entries = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        install_dir = os.path.join("share", package_name, root)
        paths = [os.path.join(root, name) for name in files]
        entries.append((install_dir, paths))
    return entries


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
        *data_files_for("web"),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn", "websockets", "PyYAML"],
    zip_safe=True,
    maintainer="kuuve",
    maintainer_email="kuuve@todo.todo",
    description="Web control panel for AUV localization tests and pinger homing.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "server = kmu26_auv_web_gui.server:main",
            "analyze_bag = kmu26_auv_web_gui.bag_analyzer:main",
            "dronecan_dynamic_id_allocator = kmu26_auv_web_gui.dronecan_dynamic_id_allocator:main",
        ],
    },
)
