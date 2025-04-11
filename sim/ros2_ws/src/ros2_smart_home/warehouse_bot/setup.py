from setuptools import setup, find_packages
import os
from glob import glob

package_name = "warehouse_bot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/warehouse_bot"]),
        ("share/" + package_name, ["package.xml"]),
        # launch 파일 자동 포함
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # config 파일 포함
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=[
        "setuptools",
        "ssafy_msgs",
    ],
    zip_safe=True,
    maintainer="SSAFY12thE102",
    maintainer_email="j1113019@naver.com",
    description="Warehouse autonomous robot core nodes",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_launch = warehouse_bot.main_launch:main",
            "run_mapping = warehouse_bot.slam.run_mapping:main",
            "auto_mapping = warehouse_bot.slam.auto_mapping_fsm:main",
            "odom = warehouse_bot.slam.odom:main",
            "a_star = warehouse_bot.navigation.a_star:main",
            "a_star_local_path = warehouse_bot.navigation.a_star_local_path:main",
            "path_tracking = warehouse_bot.controller.path_tracking:main",
            "load_map = warehouse_bot.slam.load_map:main",
            "pick_and_place_node = warehouse_bot.pick_and_place.pick_and_place_node:main",
            "precise_alignment = warehouse_bot.controller.precise_alignment:main",
            "object_detector = warehouse_bot.perception.object_detector:main",
            "ex_calib = warehouse_bot.perception.ex_calib:main",
            "mode_manager_node = warehouse_bot.mode_control.mode_manager_node:main",
            "trace_path_node = warehouse_bot.trace_path.trace_path_node:main",
        ],
    },
)
