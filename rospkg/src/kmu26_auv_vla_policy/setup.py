from setuptools import setup
from glob import glob
name = "kmu26_auv_vla_policy"
setup(name=name, version="0.1.0", packages=[name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/"+name]),
        ("share/"+name, ["package.xml", "LICENSE", "README.md", "requirements-ros.txt"]),
        ("share/"+name+"/config", glob("config/*.yaml")),
        ("share/"+name+"/launch", glob("launch/*.launch.py"))],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["policy = kmu26_auv_vla_policy.kmu26_ros:main"]})
