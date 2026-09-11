from glob import glob

from setuptools import find_packages, setup


package_name = "kmu26_auv_vla_data_collector"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kuuve",
    maintainer_email="kuuve@todo.todo",
    description="Episode-based real-world data collector for the KMU26 U0 VLA policy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "collector = kmu26_auv_vla_data_collector.collector:main",
            "export_lerobot = kmu26_auv_vla_data_collector.export_lerobot:main",
        ],
    },
)
