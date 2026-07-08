import os
from glob import glob

from setuptools import find_packages, setup

package_name = "k1muse_exploration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    scripts=[
        "scripts/k1_start_exploration.sh",
        "scripts/k1_stop_exploration.sh",
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YJY",
    maintainer_email="yjy@example.com",
    description="RRT/frontier exploration controller for K1 MUSE Pi Pro mapping.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rrt_frontier_explorer = k1muse_exploration.rrt_frontier_explorer:main",
        ],
    },
)
