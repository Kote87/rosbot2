from setuptools import setup

package_name = "rosbotxl_auto"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "loop_rect = rosbotxl_auto.loop_rect:main",
        ],
    },
)
